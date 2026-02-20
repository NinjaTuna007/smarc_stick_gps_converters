#! /bin/bash
ROBOT_NAME=stick
SESSION=${ROBOT_NAME}_bringup
USE_SIM_TIME=False

# New variables for wasp_bt.launch and wasp_mqtt_agent.launch
AGENT_TYPE=surface
PULSE_RATE=1.0 # Hz
CONTEXT=waraps # change this to 'smarc' or something else

# Acoustic modem parameters
TIMEOUT_THRESHOLD=0.5
PING_COMMAND='$P111'
TIMER_PERIOD=0.5

if [ "$USE_SIM_TIME" = "True" ]; then
    REALSIM=simulation
    LINK_SUFFIX="_gt"
else
    REALSIM=real
    LINK_SUFFIX=""
fi

# Window 1: GPS and basic sensors
tmux -2 new-session -d -s $SESSION -n 'gps_sensors'
tmux select-window -t $SESSION:0
tmux send-keys "ros2 launch ublox_gps ublox_gps_namespace.launch robot_name:=$ROBOT_NAME" C-m

# Window 2: WARAPS agent (Level 1)
tmux new-window -t $SESSION:1 -n 'waraps_agent'
tmux select-window -t $SESSION:1
tmux send-keys "ros2 launch wasp_bt wasp_mqtt_agent.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME" C-m

# Window 3: MQTT bridge
tmux new-window -t $SESSION:2 -n 'mqtt_bridge'
tmux select-window -t $SESSION:2
tmux send-keys "ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=20.240.40.232 broker_port:=1884 robot_name:=$ROBOT_NAME domain:=$AGENT_TYPE realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m

# Window 4: Acoustic modem (last window as requested)
tmux new-window -t $SESSION:3 -n 'acoustic_modem'
tmux select-window -t $SESSION:3
tmux send-keys "ros2 launch serial_ping_pkg single_target_ping_node.launch timeout_threshold:=$TIMEOUT_THRESHOLD ping_command:=\\$PING_COMMAND timer_period:=$TIMER_PERIOD robot_name:=$ROBOT_NAME" C-m

# Window 5: smarc_gps_converter
tmux new-window -t $SESSION:4 -n 'gps_converter'
tmux select-window -t $SESSION:4
tmux send-keys "ros2 launch smarc_gps_converters smarc_gps_converter.launch robot_name:=$ROBOT_NAME use_sim_time:=$USE_SIM_TIME" C-m

# Window 6: GPS to modem TF
tmux new-window -t $SESSION:5 -n 'gps_to_modem_tf'
tmux select-window -t $SESSION:5
tmux send-keys "ros2 launch smarc_gps_converters gps_to_modem_tf.launch.py world_frame:=map gps_frame:=${ROBOT_NAME}_gps modem_frame:=${ROBOT_NAME}_modem z_offset:=-1.57 gps_topic:=smarc/latlon robot_name:=$ROBOT_NAME" C-m

# Window 7: Empty window (rightmost)
tmux new-window -t $SESSION:6 -n 'empty'
tmux select-window -t $SESSION:6

# Set default window to the acoustic modem window
tmux select-window -t $SESSION:3
tmux -2 attach-session -t $SESSION
