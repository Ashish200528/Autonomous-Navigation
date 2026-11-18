#!/usr/bin/env python3
"""Bounded online-SLAM/Nav2 evaluator. Gazebo truth is evaluator-only."""
import argparse, csv, json, math, re, subprocess, time, statistics
from pathlib import Path
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from scenario_generator import point_obstacle_distance
try:
    import psutil
except ImportError:
    psutil = None


def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def quat_yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def map_goal(start_map, start_world, goal_world):
    x,y,yaw=start_map; sx,sy,syaw=start_world; gx,gy,gyaw=goal_world
    d=wrap(yaw-syaw); c,s=math.cos(d),math.sin(d); dx,dy=gx-sx,gy-sy
    return [x+c*dx-s*dy,y+s*dx+c*dy,wrap(yaw+wrap(gyaw-syaw))]
def clearance(p,layout):
    xmin,xmax,ymin,ymax=layout["arena"]["inner_bounds_xy"]
    r=float(layout["arena"]["robot_circumscribed_radius_m"])
    m=min(p["x_m"]-xmin,xmax-p["x_m"],p["y_m"]-ymin,ymax-p["y_m"])-r
    for o in layout["obstacles"]: m=min(m,point_obstacle_distance(p["x_m"],p["y_m"],o)-r)
    return m


class Eval(Node):
    def __init__(self):
        super().__init__("phase8c_lidar_nav_evaluator")
        self.map=None; self.costmap=None; self.safety=None; self.guard=None; self.odom=None
        self.map_last_wall=0.0; self.rgb_last_wall=0.0
        self.scans=[]; self.rgb=0; self.maps=0; self.costmaps=0; self.plans=0; self.plan=[]
        self.cmd={t:[] for t in ("/cmd_vel_nav_raw","/cmd_vel_smoothed","/cmd_vel_nav","/cmd_vel")}
        self.contacts=set(); self.collisions=[]; self.clock=[]; self.truth=[]
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.active_goal=None
        self.action=ActionClient(self,NavigateToPose,"/navigate_to_pose")
        self.truth_client=self.create_client(GetEntityState,"/gzserver/get_entity_state")
        self.lifecycle={n:self.create_client(GetState,f"/{n}/get_state") for n in
          ("planner_server","controller_server","behavior_server","bt_navigator","velocity_smoother","collision_monitor")}
        mapq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,"/map",self.map_cb,mapq)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",self.cost_cb,mapq)
        self.create_subscription(LaserScan,"/realistic/scan",self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Image,"/camera/image_raw",self.rgb_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,"/odometry/filtered",lambda m:setattr(self,"odom",m),qos_profile_sensor_data)
        self.create_subscription(PathMsg,"/plan",self.plan_cb,10)
        self.create_subscription(Contacts,"/contacts",self.contact_cb,qos_profile_sensor_data)
        self.create_subscription(Clock,"/clock",self.clock_cb,qos_profile_sensor_data)
        self.create_subscription(String,"/realistic/safety_state",self.safety_cb,10)
        self.create_subscription(String,"/realistic/command_guard_state",self.guard_cb,10)
        for t in self.cmd: self.create_subscription(Twist,t,lambda m,t=t:self.cmd[t].append((time.monotonic(),m.linear.x,m.angular.z)),10)
    def map_cb(self,m): self.map=m; self.maps+=1; self.map_last_wall=time.monotonic()
    def rgb_cb(self,m): self.rgb+=1; self.rgb_last_wall=time.monotonic()
    def cost_cb(self,m): self.costmap=m; self.costmaps+=1
    def scan_cb(self,m): self.scans.append((time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9))
    def plan_cb(self,m): self.plan=[(p.pose.position.x,p.pose.position.y) for p in m.poses]; self.plans+=1
    def safety_cb(self,m):
        try:self.safety=json.loads(m.data)
        except Exception:self.safety={"valid":False,"reason":"malformed"}
    def guard_cb(self,m):
        try:self.guard=json.loads(m.data)
        except Exception:self.guard={"authorized":False}
    def clock_cb(self,m): self.clock.append((time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9))
    def contact_cb(self,m):
        curr=set()
        for c in m.contacts:
            a,b=str(c.collision1.name),str(c.collision2.name)
            if any(x in (a+" "+b).lower() for x in ("ground_plane","ground::","floor")): continue
            if a and b: curr.add(tuple(sorted((a,b))))
        for p in curr-self.contacts:self.collisions.append(p)
        self.contacts=curr
    def pose_map(self):
        t=self.tf.lookup_transform("map","base_footprint",Time()).transform
        return (t.translation.x,t.translation.y,quat_yaw(t.rotation))
    def goal_map_pose(self,map_start,start_world,goal_world):
        return map_goal(getattr(self,"fixed_goal_map_reference",map_start),start_world,goal_world)

    def get_truth(self):
        f=self.truth_client.call_async(GetEntityState.Request(entity="anc_robot"))
        end=time.monotonic()+2.5
        while rclpy.ok() and not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
        if not f.done() or f.result() is None or f.result().result.result!=1:raise RuntimeError("evaluator_truth_unavailable")
        s=f.result().state
        p={"wall_s":time.monotonic(),"sim_s":s.header.stamp.sec+s.header.stamp.nanosec*1e-9,
           "x_m":s.pose.position.x,"y_m":s.pose.position.y,"yaw_rad":quat_yaw(s.pose.orientation),
           "linear_x_mps":s.twist.linear.x,"angular_z_radps":s.twist.angular.z}
        self.truth.append(p); return p
    def lifecycle_states(self):
        out={}
        for n,c in self.lifecycle.items():
            if not c.service_is_ready():out[n]="service_missing";continue
            f=c.call_async(GetState.Request()); end=time.monotonic()+1
            while rclpy.ok() and not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
            out[n]=f.result().current_state.label if f.done() and f.result() else "timeout"
        return out
    def truth_ready(self, timeout=.1):
        return self.truth_client.wait_for_service(timeout_sec=timeout)

    def ready(self,timeout=35):
        end=time.monotonic()+timeout; last="map_tf_nav2_not_ready"
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            try: pose=self.pose_map()
            except Exception: pose=None
            known=0 if self.map is None else sum(v>=0 for v in self.map.data)
            fresh=bool(self.scans and time.monotonic()-self.scans[-1][0]<.5)
            rgb_fresh=self.rgb>0 and time.monotonic()-self.rgb_last_wall<.7
            if self.map is not None and known>=200 and fresh and rgb_fresh and self.odom and pose and self.safety:
                states=self.lifecycle_states()
                if all(v=="active" for v in states.values()) and self.action.wait_for_server(timeout_sec=.1) and self.truth_ready(.1):
                    if self.safety.get("valid") and self.safety.get("max_linear_speed_mps",0)>0:return pose,states
                    last="safety_gate_no_forward_authority:"+str(self.safety.get("reason"))
                else:last="nav2_not_active:"+json.dumps(states,sort_keys=True)
        raise RuntimeError(last)
    def run(self,layout,timeout_s):
        map_start,states=self.ready()
        start=self.get_truth(); sw=list(map(float,layout["start_xyyaw"])); gw=list(map(float,layout["goal_xyyaw"]))
        gm=self.goal_map_pose(map_start,sw,gw)
        g=NavigateToPose.Goal(); g.pose=PoseStamped(); g.pose.header.frame_id="map"
        g.pose.header.stamp=self.get_clock().now().to_msg(); g.pose.pose.position.x=gm[0]; g.pose.pose.position.y=gm[1]
        g.pose.pose.orientation.z=math.sin(gm[2]/2); g.pose.pose.orientation.w=math.cos(gm[2]/2)
        f=self.action.send_goal_async(g); end=time.monotonic()+8
        while rclpy.ok() and not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
        if not f.done() or not f.result() or not f.result().accepted:raise RuntimeError("nav2_goal_not_accepted")
        gh=f.result(); self.active_goal=gh; rf=gh.get_result_async(); t0=time.monotonic(); last_truth=0; timeout=min(timeout_s,45 if not layout.get("route_expected",True) else timeout_s)
        limit=max(8,math.hypot(gw[0]-sw[0],gw[1]-sw[1])+2); over=False; resources=[]
        while rclpy.ok() and not rf.done() and time.monotonic()-t0<timeout:
            rclpy.spin_once(self,timeout_sec=.05); now=time.monotonic()
            if self.collisions:
                gh.cancel_goal_async()
                raise RuntimeError('non_ground_contact_abort')
            available=int(next(l.split()[1] for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith('MemAvailable:')))
            if available<768*1024:
                gh.cancel_goal_async()
                raise RuntimeError('commission_memory_abort')
            if now-last_truth>.35:
                p=self.get_truth(); last_truth=time.monotonic()
                if math.hypot(p["x_m"]-start["x_m"],p["y_m"]-start["y_m"])>limit:
                    over=True; gh.cancel_goal_async(); break
            if int(now-t0)>len(resources):resources.append(self.resource_sample())
        timed=not rf.done()
        if timed:gh.cancel_goal_async()
        status="TIMEOUT" if timed else {GoalStatus.STATUS_SUCCEEDED:"SUCCEEDED",GoalStatus.STATUS_ABORTED:"ABORTED",GoalStatus.STATUS_CANCELED:"CANCELED"}.get(rf.result().status,str(rf.result().status))
        end=time.monotonic()+1.5
        while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
        final=self.get_truth(); err=math.hypot(final["x_m"]-gw[0],final["y_m"]-gw[1]); herr=abs(wrap(final["yaw_rad"]-gw[2]))
        plen=sum(math.hypot(b["x_m"]-a["x_m"],b["y_m"]-a["y_m"]) for a,b in zip(self.truth[:-1],self.truth[1:]))
        clear=min((clearance(p,layout) for p in self.truth),default=None)
        no_route=not bool(layout.get("route_expected",True))
        pubs={t:self.publisher_count(t) for t in ("/cmd_vel","/cmd_vel_nav","/cmd_vel_nav_raw","/cmd_vel_smoothed")}
        sole_guard=self.is_sole_guard_publisher(pubs.get("/cmd_vel",{}))
        now=time.monotonic()
        sensor_integrity={
          "rgb_frame_count":self.rgb,
          "rgb_fresh":self.rgb>0 and now-self.rgb_last_wall<.7,
          "scan_fresh":bool(self.scans and now-self.scans[-1][0]<.5),
          "map_fresh":self.map is not None and now-self.map_last_wall<4.0,
          "filtered_odom_received":self.odom is not None,
        }
        command={}
        for t,vals in self.cmd.items():
            command[t]={"count":len(vals),"nonzero":sum(abs(v)>1e-6 or abs(w)>1e-6 for _,v,w in vals),
                        "max_v":max((abs(v) for _,v,w in vals),default=0),"max_w":max((abs(w) for _,v,w in vals),default=0)}
        guard_output_observed=command["/cmd_vel"]["count"]>0
        sensors_fresh=all((sensor_integrity["rgb_fresh"],sensor_integrity["scan_fresh"],
                           sensor_integrity["map_fresh"],sensor_integrity["filtered_odom_received"]))
        passed=((status=="ABORTED" and plen<=.10 and not self.collisions and not over) if no_route else
                (status=="SUCCEEDED" and err<=.30 and herr<=.30 and clear is not None and clear>=0 and not self.collisions and not over))
        passed=passed and sole_guard and guard_output_observed and sensors_fresh
        return {"classification":"REALISTIC_RGB_LIDAR_NAV2_GOAL_TRIAL","status":"PASS" if passed else "FAIL",
          "scenario_id":layout["scenario_id"],"seed":int(layout["seed"]),"route_expected":not no_route,
          "runtime_navigation_receives_no_gazebo_truth":True,"localization_mode":"known-start online SLAM",
          "start_world_pose":sw,"goal_world_pose":gw,"start_map_pose":map_start,"goal_map_pose":gm,
          "start_truth":start,"final_truth":final,"final_xy_error_m":err,"final_heading_error_rad":herr,
          "truth_path_length_m":plen,"min_evaluator_clearance_m":clear,"nav2_action_status":status,
          "travel_limit_exceeded":over,"elapsed_wall_s":time.monotonic()-t0,
          "simulated_elapsed_s":max(0,final["sim_s"]-start["sim_s"]),
          "sensor_integrity":sensor_integrity,"sole_authoritative_cmd_vel_publisher":sole_guard,
          "sensor_summary":{"scan_count":len(self.scans),"rgb_frames":self.rgb,"map_updates":self.maps,
            "global_costmap_updates":self.costmaps,"plan_messages":self.plans,
            "scan_rate_hz":None if len(self.scans)<2 else (len(self.scans)-1)/max(1e-9,self.scans[-1][1]-self.scans[0][1])},
          "safety_state":self.safety,"guard_state":self.guard,"nav2_lifecycle_states":states,
          "command_summary":command,"cmd_vel_publisher_audit":pubs,"non_ground_contact_events":self.collisions,
          "resource_samples":resources,"resource_summary":self.resource_summary(resources),"runtime_rtf":None if len(self.clock)<2 else (self.clock[-1][1]-self.clock[0][1])/max(1e-9,self.clock[-1][0]-self.clock[0][0]),
          "acceptance":{"xy_error_limit_m":.30,"heading_error_limit_rad":.30,"zero_collisions":True,"unknown_not_plannable":True}}
    def resource_sample(self):
        mem={}
        try:
            for line in Path("/proc/meminfo").read_text().splitlines():
                key, value = line.split(":", 1)
                if key in ("MemAvailable", "SwapFree"):
                    mem[key.lower()+"_kib"] = int(value.split()[0])
        except Exception:
            pass
        rss_bytes=0; cpu_pct=0.0; matched=[]
        patterns=("gz sim","gzserver","ros_gz_bridge","slam_toolbox","nav2_",
                  "controller_server","planner_server","behavior_server","bt_navigator",
                  "velocity_smoother","collision_monitor","anc_lidar","anc_perception",
                  "anc_realistic_navigation","robot_state_publisher","robot_localization",
                  "ekf_node","ros2 launch anc_lidar","phase8c_navigation_runner")
        if psutil is not None:
            try:
                for proc in psutil.process_iter(["cmdline"]):
                    try:
                        cmd=" ".join(proc.info.get("cmdline") or []).lower()
                        if any(pattern in cmd for pattern in patterns):
                            rss_bytes += proc.memory_info().rss
                            cpu_pct += proc.cpu_percent(interval=None)
                            matched.append(proc.pid)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
            except Exception:
                pass
        return {"wall_s":time.monotonic(), **mem,
                "target_process_count":len(matched),
                "target_process_rss_mib":round(rss_bytes/(1024*1024),2),
                "target_process_cpu_percent":round(cpu_pct,2) if psutil is not None else None,
                "wsl_cpu_percent":round(psutil.cpu_percent(interval=None),2) if psutil is not None else None}

    @staticmethod
    def resource_summary(samples):
        rss=[x["target_process_rss_mib"] for x in samples if x.get("target_process_rss_mib") is not None]
        cpu=[x["target_process_cpu_percent"] for x in samples if x.get("target_process_cpu_percent") is not None]
        wsl=[x["wsl_cpu_percent"] for x in samples if x.get("wsl_cpu_percent") is not None]
        return {"scope":"best-effort processes matching the ROS/Gazebo/Nav2 command-name patterns; excludes Windows-only processes",
                "psutil_available":psutil is not None,
                "target_process_peak_rss_mib":max(rss) if rss else None,
                "target_process_mean_rss_mib":round(statistics.mean(rss),2) if rss else None,
                "target_process_mean_cpu_percent":round(statistics.mean(cpu),2) if cpu else None,
                "wsl_mean_cpu_percent":round(statistics.mean(wsl),2) if wsl else None,
                "sample_count":len(samples)}
    @staticmethod
    def parse_publisher_info(output):
        match=re.search(r"Publisher count:\s*(\d+)",output)
        publisher_section=output.split("Subscription count:",1)[0]
        names=re.findall(r"Node name:\s*(\S+)",publisher_section)
        return {"count":int(match.group(1)) if match else None,"publisher_names":names}

    @staticmethod
    def is_sole_guard_publisher(info):
        names=[str(x).strip("/").split("/")[-1] for x in info.get("publisher_names",[])]
        return info.get("count")==1 and names==["anc_safety_command_guard"]

    def publisher_count(self,topic):
        # Query this node's already-discovered graph without blocking sensor callbacks
        # on multiple external CLI/daemon processes.
        try:
            publishers=self.get_publishers_info_by_topic(topic)
            return dict(count=len(publishers),publisher_names=[p.node_name for p in publishers],
                        audit_method='rclpy_graph')
        except Exception as exc:
            return dict(error=str(exc),count=None,publisher_names=[])


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("--layout",type=Path,required=True); a.add_argument("--output-dir",type=Path,required=True)
    a.add_argument("--timeout-s",type=float,default=90); x=a.parse_args()
    layout=json.loads(x.layout.read_text()); x.output_dir.mkdir(parents=True,exist_ok=True); rclpy.init(); node=Eval()
    try:
        report=node.run(layout,x.timeout_s); code=0 if report["status"]=="PASS" else 2
    except Exception as e:
        report={"classification":"REALISTIC_RGB_LIDAR_NAV2_GOAL_TRIAL","status":"FAIL","scenario_id":layout.get("scenario_id"),
                "seed":layout.get("seed"),"reason":f"{type(e).__name__}: {e}","safety_state":node.safety,"guard_state":node.guard,
                "runtime_navigation_receives_no_gazebo_truth":True}; code=2
    report["layout_manifest"]=str(x.layout); report["evaluator_world_geometry_used_by_navigation"]=False
    (x.output_dir/"navigation-result.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n")
    with (x.output_dir/"truth-trajectory.csv").open("w",newline="") as h:
        fields=["wall_s","sim_s","x_m","y_m","yaw_rad","linear_x_mps","angular_z_radps"]
        w=csv.DictWriter(h,fieldnames=fields); w.writeheader(); w.writerows(node.truth)
    print(json.dumps(report,indent=2,sort_keys=True)); node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    return code

if __name__=="__main__":raise SystemExit(main())
