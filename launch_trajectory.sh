#!/bin/zsh

while getopts "x:y:" opt; do 
    case $opt in
        x) x_pos=$OPTARG ;;
        y) y_pos=$OPTARG ;;
    esac
done

SETUP_ENV='
export PATH="$HOME/.pixi/bin:$PATH"
source ~/.zshrc
cd /home/neverorfrog/code/crazyflie/crazy-neural-mpc
'

cleanup() {
    echo "Cleaning up..."
    killall -9 gzserver gzclient ruby rviz2 2>/dev/null
    ros2 daemon stop
    exit 0
}
trap cleanup SIGINT

tilix -a app-new-session -e "bash -c 'cd external/crazyflie-firmware && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x $x_pos -y $y_pos; exec bash'"
tilix -a session-add-right -e "zsh -ic 'eval \"$SETUP_ENV\" && pixi run mpc; exec zsh'"
sleep 7

tilix -a session-add-down -e "zsh -ic 'eval \"$SETUP_ENV\" && pixi run swarm; exec zsh'"
sleep 3

# Start recording in new tab
tilix -a session-add-down -e "zsh -ic 'eval \"$SETUP_ENV\" && pixi run record; exec zsh'"
tilix -a session-add-right -e "zsh -ic 'eval \"$SETUP_ENV\" && pixi run mpc-takeoff; exec zsh'"
sleep 4
tilix -a session-add-right -e "zsh -ic 'eval \"$SETUP_ENV\" && pixi run mpc-traj; exec zsh'"

