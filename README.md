ROS2 Package for Localization and Navigation Goal Event Capture
This package is designed to monitor and report significant events related to robot localization and navigation, leveraging the ROS2 framework. It consists of two interconnected components: one for evaluating localization quality through AMCL particle dispersion and another for capturing and displaying navigation goal events.

1. Localization Event Monitoring (lost_event.py)
Overview
The lost_event.py script assesses the quality of robot localization by analyzing the dispersion of AMCL particles. The state of localization is inferred from the spread of these particles; a narrow dispersion indicates precise localization, while a wide dispersion suggests uncertainty.

Localization States
The script classifies localization quality into three categories:

GOOD: Localization is accurate, with particles densely clustered.
BAD: Localization is imprecise, with a moderate spread of particles.
LOST: Localization is unreliable, with particles widely dispersed.
Execution
Run lost_event.py in its directory to start monitoring localization events:

bash
Copy code
python3 lost_event.py
This script subscribes to the "/localization_precision" topic to gather necessary data for evaluating localization quality.

2. Navigation Goal Event Capture
Building upon the "/localization_precision" data, this component implements logic to capture and display events related to the robot's navigation goals.

Features
Starting Point: The location from where the robot initiates its journey.
End Point: The robot's intended destination.
Initial Trajectory Path Length: The length of the planned path to the destination, in meters.
Remaining Length of Trajectory: Updated in real-time as the robot progresses, shown in meters and as a percentage of the total path length.
Navigation Status:
STATUS_SUCCEEDED: The robot has reached its destination successfully.
STATUS_CANCELED: The navigation goal was canceled.
STATUS_ABORTED: The navigation was aborted due to complications.
Recoveries: Counts the number of recovery maneuvers performed in response to navigational challenges.
Length from Last Moved Point to Goal: The distance from the robot's last known position to the end point, in meters.
Current Position of Robot: The robot's current location during its journey.
Integration and Usage
This functionality relies on the data provided by /localization_precision and additional logic for event capture. It seamlessly integrates with the lost_event.py script for a comprehensive monitoring solution. Ensure your ROS2 environment is correctly set up to subscribe to both the "/localization_precision" topic and any other relevant navigation topics for this feature to function correctly.

