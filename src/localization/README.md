# localization
ROS2 localization nodes for teaching "Control and Robot Programming" course (CyPR) at Universidad e Málaga (Spain)

This package contains two nodes:
LSE: Robot localization based on least square errors, and range only measurements to N landmarks.
PF: Robot localizatio based on a particle filter and range and bearing measurements to a single landmark.

Note: These nodes are designed to work with the provided CoppeliaSim scenes, where the range&bearing sensor is simulated.