Values characterizing your robot:
Characterization Tutorial: https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html

1. Open your characterization tool.
	- This will often be accessed through your terminal/cmd/powershell/etc. 
	- Windows systems, and most likely all others, but im not sure, will use the command '**frc-characterization drive new**' in order to launch/open the tool.
	- Of course, this requires that you have all necessary code/programs/languages set up (python, frc game tools, etc.)
2. Set up your characterization tool.
	- Select your current *robotconfig.py* file using the button in the top left. You can create a new one in a chosen location, if you have not already made one. 
	- *Team Number*: 8114
	- *Unit Type*: Meters
	- *Units per Rotation*: 0.478778720406999
	- *Project Type (Should be automatic)*: Drivetrain
	- *Control Type*: Spark Max
3. Set the correct values for your *robotconfig.py* file.
	- *brushed*: False
	- *motorPorts*: [2, 1]
	- *rightMotorPorts*: [3, 4]
	- *motorsInverted*: [False, False]
	- *rightMotorsInverted*: [False, False]
	- *encoderEPR*: 1
	- *gearing*: 10.75
	- *useDataPort*: False
	- *encoderPorts*: []
	- *rightEncoderPorts*: []
	- *encoderInverted*: False
	- *rightEncoderInverted*: False
	- *gyroType*: ADXRS450
	- *gyroPort*: SPI.Port.kMXP
4. Reference characterization tutorial if needed.
