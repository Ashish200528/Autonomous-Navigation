env --default-signal=INT,QUIT setsid --wait bash "$1" "$2" "$3" &
child_pid=$!
printf "%s\n" "$child_pid" > "$4"
wait "$child_pid"
