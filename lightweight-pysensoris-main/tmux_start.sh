#!/bin/sh

# Set the session name
SESSION="SensorTraffix"

# Start a new tmux session and detach from it
tmux new-session -d -s $SESSION -n 'Sensors'

tmux source-file ./tmux.conf

# Start the first process in the first pane
tmux send-keys -t $SESSION 'source ../../devel/setup.bash' C-m
tmux send-keys -t $SESSION 'python3 main_sending.py -ip 192.168.199.80 -pt 1883 -tn /Global/DetectedObjects -nn sensoris_listener_gf -mt sensoris_5GoIng/mqtt/global_objects' C-m

# Create new panes and run the other processes
for i in $(seq 12 22)
do
    # Create a new pane by splitting vertically and adjust the layout immediately
    tmux split-window -v -t $SESSION
    tmux select-layout -t $SESSION tiled

    # Wait a bit to ensure the pane is ready (optional, for slower systems)
    sleep 0.5

    # Send the source and python commands to the new pane
    tmux send-keys -t $SESSION "source ../../devel/setup.bash" C-m
    tmux send-keys -t $SESSION "python3 main_mqtt_sub.py -ip 192.168.199.80 -pt 1883 -tn mast_$i/fused_tracked_objects -nn sensoris_listener_$i -mt sensoris_5GoIng/mqtt/mast_$i/fused_tracked_objects" C-m
done

# Add a new pane for the roslaunch command
tmux split-window -v -t $SESSION
tmux select-layout -t $SESSION tiled

# Send the roslaunch command to the new pane
tmux send-keys -t $SESSION 'source ../../devel/setup.bash' C-m
tmux send-keys -t $SESSION 'roslaunch ../../launch/Global_Fusion.launch' C-m

# Additional Pane 1
tmux split-window -v -t $SESSION
tmux select-layout -t $SESSION tiled
tmux send-keys -t $SESSION 'echo "Pane 1 running"' C-m

# Additional Pane 2
tmux split-window -v -t $SESSION
tmux select-layout -t $SESSION tiled
tmux send-keys -t $SESSION 'echo "Pane 2 running"' C-m

# Additional Pane 3
tmux split-window -v -t $SESSION
tmux select-layout -t $SESSION tiled
tmux send-keys -t $SESSION 'echo "Pane 3 running"' C-m

# After setting up all panes, choose how to display them
tmux select-layout -t $SESSION tiled

# Attach to the session
tmux attach -t $SESSION

