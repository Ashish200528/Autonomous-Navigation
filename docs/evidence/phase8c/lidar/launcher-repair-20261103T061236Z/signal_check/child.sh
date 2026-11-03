trap 'printf "INT_TRAPPED\n" > "$1"; exit 0' INT
printf "READY\n" > "$2"
while :; do sleep 0.1; done
