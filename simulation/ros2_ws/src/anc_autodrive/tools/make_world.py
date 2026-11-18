"""Generate worlds/autodrive_track.sdf (output is committed).

    python tools/make_world.py

A 1:5 scale town around the track (lane 0.6 m = 3 m real). People, cars, the stop
sign and the hydrant are Gazebo Fuel models (CC-BY 4.0, OpenRobotics) scaled by S so
YOLO recognises them and the vision node's height priors work with object_scale=S.
Rules for placement:
  * in-lane obstacles only on straights (the overtake manoeuvre assumes a straight road)
  * nothing coloured in the infield: the left-lane check looks there before overtaking
  * roadside objects >= 0.45 m outside the white edge so the ultrasonic never sees them
"""
import math
from pathlib import Path

S = 0.2  # world scale for real-world models
FUEL = "https://fuel.gazebosim.org/1.0/openrobotics/models"
OUT = Path(__file__).resolve().parent.parent / "worlds" / "autodrive_track.sdf"


def mat(r, g, b, emissive=None):
    e = f"<emissive>{emissive}</emissive>" if emissive else ""
    return f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>{e}</material>"


def box(name, pose, size, material):
    return f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>{material}</visual>'


def cyl(name, pose, r, length, material):
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><cylinder><radius>{r}</radius>'
            f'<length>{length}</length></cylinder></geometry>{material}</visual>')


def static(name, x, y, yaw, body, collision=""):
    return (f'    <model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose>'
            f'<link name="link">{body}{collision}</link></model>\n')


def fuel_mesh(name, x, y, yaw, path, scale, mesh_pose="0 0 0 0 0 0", collision=""):
    v = (f'<visual name="mesh"><pose>{mesh_pose}</pose><geometry><mesh><uri>{FUEL}/{path}</uri>'
         f'<scale>{scale} {scale} {scale}</scale></mesh></geometry></visual>')
    return static(name, x, y, yaw, v, collision)


def barrier(name, x, y):
    body = "".join([
        box("base", "0 0 0.02 0 0 0", "0.16 0.40 0.04", mat(0.12, 0.12, 0.13)),
        box("body", "0 0 0.17 0 0 0", "0.12 0.36 0.26", mat(0.95, 0.18, 0.04)),
        box("band1", "0 0 0.13 0 0 0", "0.125 0.365 0.035", mat(0.96, 0.96, 0.96)),
        box("band2", "0 0 0.22 0 0 0", "0.125 0.365 0.035", mat(0.96, 0.96, 0.96)),
        cyl("lamp", "0 0.12 0.32 0 0 0", 0.022, 0.03, mat(1, 0.65, 0, "1 0.55 0 1")),
    ])
    col = '<collision name="c"><pose>0 0 0.15 0 0 0</pose><geometry><box><size>0.12 0.36 0.30</size></box></geometry></collision>'
    return static(name, x, y, 0, body, col)


def stop_sign(x, y):
    """Own octagon mesh + texture (the Fuel stop sign's OGRE-1 material renders blank white).
    0.15 m across = a 0.75 m real sign at 1:5; faces -X, i.e. traffic coming along +X."""
    tex = ('<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal>'
           '<albedo_map>model://autodrive_track/materials/textures/stop_sign.png</albedo_map>'
           '<roughness>0.6</roughness><metalness>0</metalness></metal></pbr></material>')
    body = (cyl("pole", "0.004 0 0.17 0 0 0", 0.008, 0.34, mat(0.55, 0.57, 0.6))
            + f'<visual name="sign"><pose>0 0 0.36 0 0 0</pose><geometry><mesh>'
              f'<uri>model://autodrive_track/meshes/stop_sign.obj</uri></mesh></geometry>{tex}</visual>')
    return static("stop_sign", x, y, 3.1416, body)


def lamp(name, x, y, side):
    """Street lamp on the sidewalk; the arm reaches over the road (side = +1 north, -1 south)."""
    arm_y = -side * 0.09
    body = "".join([
        cyl("pole", "0 0 0.3 0 0 0", 0.012, 0.6, mat(0.18, 0.19, 0.21)),
        box("arm", f"0 {arm_y} 0.6 0 0 0", "0.02 0.2 0.015", mat(0.18, 0.19, 0.21)),
        box("head", f"0 {2 * arm_y} 0.59 0 0 0", "0.07 0.05 0.02", mat(0.18, 0.19, 0.21)),
        box("glow", f"0 {2 * arm_y} 0.578 0 0 0", "0.06 0.04 0.004", mat(1, 0.95, 0.8, "1 0.92 0.7 1")),
    ])
    return static(name, x, y, 0, body)


def building(name, x, y, w, d, h, facade, roof):
    tex = (f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal>'
           f'<albedo_map>model://autodrive_track/materials/textures/{facade}.png</albedo_map>'
           f'<roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material>')
    body = (box("walls", f"0 0 {h / 2} 0 0 0", f"{w} {d} {h}", tex)
            + box("roof", f"0 0 {h + 0.015} 0 0 0", f"{w + 0.04} {d + 0.04} 0.03", mat(*roof)))
    return static(name, x, y, 0, body)


def pine(name, x, y, k=1.0):
    body = (cyl("trunk", f"0 0 {0.2 * k} 0 0 0", 0.05 * k, 0.4 * k, mat(0.33, 0.2, 0.1))
            + f'<visual name="crown"><pose>0 0 {0.85 * k} 0 0 0</pose><geometry><cone><radius>{0.38 * k}</radius>'
              f'<length>{1.1 * k}</length></cone></geometry>{mat(0.07, 0.32, 0.14)}</visual>')
    return static(name, x, y, 0, body)


def walker():
    """Pedestrian for the zebra crossing (x=-0.3). Gazebo's skinned actors hang the CPU-only
    renderer, so this is a rigid person model moved by VelocityControl; the sim-only
    pedestrian_sim node walks it across, waits, turns round and walks back.
    Visual only (no collision, no gravity): VelocityControl has unlimited force and would
    shove the robot off the road. Camera and ultrasonic are rendered, so both still see it."""
    vis = (f'<visual name="mesh"><pose>0 0 0 0 0 -1.5708</pose><geometry><mesh>'
           f'<uri>{FUEL}/casual female/4/files/meshes/casual_female.dae</uri><scale>{S} {S} {S}</scale>'
           f'</mesh></geometry></visual>')
    inertial = ('<inertial><pose>0 0 0.03 0 0 0</pose><mass>5</mass><inertia><ixx>1</ixx><iyy>1</iyy>'
                '<izz>0.2</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
    return (f'    <model name="pedestrian"><pose>-0.3 -4.15 0.005 0 0 1.5708</pose>'
            f'<link name="link"><gravity>false</gravity>{inertial}{vis}</link>'
            f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>'
            f'<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
            f'<odom_publish_frequency>20</odom_publish_frequency><dimensions>2</dimensions></plugin>'
            '</model>\n')


HEADER = '''<?xml version="1.0"?>
<sdf version="1.9">
  <!-- GENERATED by tools/make_world.py - edit that script, not this file.
       1:5 town around a stadium loop; the car drives counter-clockwise in the outer
       (right-hand) lane at radius 3.3 m. Default start: (-3.6, -3.3), yaw 0. -->
  <world name="anc_autodrive">
    <physics name="autodrive_physics" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.5 0.5 0.53 1</ambient>
      <background>0.55 0.72 0.93 1</background>
      <shadows>{shadows}</shadows>
      <grid>false</grid>
      <sky></sky>
    </scene>
    <!-- Window layout lives in config/autodrive_gui.config (dark theme, HUD, chase camera). -->

    <light name="sun" type="directional">
      <cast_shadows>{shadows}</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.93 0.86 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.45 0.35 -0.82</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="ground">
        <collision name="ground_collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="grass">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>
            <pbr><metal>
              <albedo_map>model://autodrive_track/materials/textures/grass.png</albedo_map>
              <roughness>0.95</roughness><metalness>0.0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <include><uri>model://autodrive_track</uri><name>track</name></include>
    <!-- Station: ArUco id 0 on the north sidewalk, facing oncoming (-X travelling) traffic. -->
    <include><uri>model://aruco_station</uri><name>station</name><pose>-2.5 3.9 0 0 0 0</pose></include>
'''


def world(shadows=True):
    parts = [HEADER.replace("{shadows}", str(shadows).lower())]
    car_col = '<collision name="c"><pose>0 0 0.1 0 0 0</pose><geometry><box><size>0.85 0.34 0.2</size></box></geometry></collision>'

    parts.append("    <!-- South straight (car travels +X): STOP sign, zebra crossing, barrier. -->\n")
    parts.append(stop_sign(-0.75, -3.86))
    parts.append(fuel_mesh("fire_hydrant", -1.7, -3.82, 0, "fire hydrant/3/files/meshes/fire_hydrant.dae", S))
    parts.append(walker())
    parts.append(barrier("barrier_south", 1.6, -3.3))

    parts.append("    <!-- North straight (car travels -X): broken-down car with a warning cone. -->\n")
    parts.append(fuel_mesh("broken_down_car", 1.2, 3.3, 0, "hatchback red/4/files/meshes/hatchback.obj",
                           0.0254 * S, "0 0 0 0 0 1.5708", car_col))
    parts.append(fuel_mesh("warning_cone", 1.95, 3.3, 0,
                           "construction cone/3/files/meshes/construction_cone.dae", 10 * S))

    parts.append("    <!-- People on the east curve sidewalk, parked SUV by the west curve. -->\n")
    for i, (a, path) in enumerate(((-32, "casual female/4/files/meshes/casual_female.dae"),
                                   (22, "standing person/3/files/meshes/standing.dae"))):
        r = 4.35
        x, y = 4 + r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        yaw = math.atan2(-y, 4 - x) - 1.5708  # face the road
        parts.append(fuel_mesh(f"bystander_{i}", round(x, 3), round(y, 3), round(yaw, 3), path, S))
    parts.append(fuel_mesh("parked_suv", -8.55, -1.0, 1.5708, "suv/4/files/meshes/suv.obj",
                           0.06 * S, "0 0 0 0 0 -1.5708"))

    parts.append("    <!-- Street furniture and town. -->\n")
    for i, x in enumerate((-3.3, -1.3, 0.9, 3.2)):
        parts.append(lamp(f"lamp_n{i}", x, 3.98, 1))
    for i, x in enumerate((-3.2, 0.45, 3.2)):
        parts.append(lamp(f"lamp_s{i}", x, -3.98, -1))
    for i, (x, w, d, h, f, roof) in enumerate((
            (-3.4, 1.0, 0.7, 0.75, "facade_sand", (0.45, 0.18, 0.15)),
            (-2.0, 0.8, 0.6, 1.05, "facade_slate", (0.2, 0.22, 0.25)),
            (-0.5, 1.2, 0.7, 0.6, "facade_sand", (0.45, 0.18, 0.15)),
            (1.3, 0.9, 0.6, 0.9, "facade_slate", (0.2, 0.22, 0.25)),
            (3.0, 1.1, 0.7, 0.7, "facade_sand", (0.45, 0.18, 0.15)))):
        parts.append(building(f"building_n{i}", x, 4.55 + d / 2, w, d, h, f, roof))
    for i, (x, w, h, f) in enumerate(((-3.0, 1.0, 0.8, "facade_slate"), (2.7, 1.3, 0.65, "facade_sand"))):
        parts.append(building(f"building_s{i}", x, -4.95, w, 0.7, h, f, (0.2, 0.22, 0.25)))
    for i, (x, y, k) in enumerate(((-7.4, -4.6, 1.1), (-5.6, -5.4, 0.9), (0.6, -5.5, 1.0), (5.0, -5.4, 1.2),
                                   (8.4, -3.2, 1.0), (8.9, 3.0, 1.1), (5.6, 5.5, 0.9), (-5.4, 5.4, 1.2),
                                   (-8.5, 3.4, 1.0), (-8.9, -3.6, 0.8))):
        parts.append(pine(f"pine_{i}", x, y, k))
    parts.append("  </world>\n</sdf>\n")
    return "".join(parts)


if __name__ == "__main__":
    # Measured with GUI + YOLO on a CPU-only (Docker/llvmpipe) host: default ~80 % real time,
    # lite ~88 %, ultra ~12 % (shadows are only worth it with a real GPU).
    OUT.write_text(world(shadows=False))
    OUT.with_name("autodrive_track_lite.sdf").write_text(
        world(shadows=False).replace("      <sky></sky>\n", ""))
    OUT.with_name("autodrive_track_ultra.sdf").write_text(world(shadows=True))
    print("wrote", OUT, "+ _lite and _ultra variants")
