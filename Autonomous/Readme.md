# Autonomous Subsystem

For running nodes using launch file you can uncomment code for launch file. I was facing an issue where these nodes were getting initialized but they were not subscribing anything.
So, what you can do is run all nodes individually using python3 in src folder. Its sequence should be path_planner.py -> color_detection.py -> controller.py.
This will start all nodes and robot will go to sites for site detection.
