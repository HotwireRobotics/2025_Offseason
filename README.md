> ![logo](src\main\deploy\logo.png)
# Hotwire Robotics Offseason/2026
## Radio Config
> ### `Configuring the Radio` 
> **1.** Open `http://10.29.90.1/configuration`     
> **2.** Set an SSID suffix: `Hotwire` and WPA keys: `TEAM2990`     
**3.** Press `Configure`
## Joystick Controls
> ### `Driver Controls`     
> **A**: Zero robot at current pose.      
**Back**: Abort current processes and return to `Default` state.      
#
> ### `Operator Controls`        
> **X**: Lower wrist.     
**Y**: Run intake motors **and** lower wrist.      
**Right Bumper**: Locate the nearest ***righthand*** branch and navigate to that position before scoring **L3**.      
**Left Bumper**: Locate the nearest ***lefthand*** branch and navigate to that position before scoring **L3**.        
**Right Trigger**: Locate the nearest ***righthand*** branch and navigate to that position before scoring **L2**.           
**Left Trigger**: Locate the nearest         ***lefthand*** branch and navigate to that position before scoring **L2**.       
**Back**: Abort current processes and return to `Default` state.    
**POV Down**: Navigate to and remove the nearest upper algae.
#
`Note: Always ensure the robot has a limelight-confirmed pose before navigating.`
## Robot Superstates
> `STOPPED` Robot is offline and all subsystems are similarly disabled.
> `HOME` Robot is in teleop; the driver is in control and all subsystems are in their default states.   
> `LOWERING` The wrist component is being relocated to `INTAKE` position.      
<!-- > `SCORING_{RIGHT/LEFT}_{UP/DOWN}` -->
