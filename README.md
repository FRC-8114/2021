Code for the Boerne Rocketeers 2021 Infinite Recharge from Home robot.

For the best experience navigating and editing the .md files in this repo, we suggest Obsidian: https://obsidian.md (note the school blocks this link) or https://github.com/obsidianmd/obsidian-releases.

**Documentation:**
- [Spark MAX Documentation](https://www.revrobotics.com/sparkmax-software/)
- [WPILib Documentation](https://docs.wpilib.org/en/stable/index.html)
	- [Trajectories Tutorial](https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/index.html)
	- [Robot Characterization Documentation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/index.html)
	- [WPILibPi](https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/index.html)
- [WPILib API](https://first.wpi.edu/FRC/roborio/release/docs/java/)
- [Infinite Recharge At Home Game Manual](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system)

[[robot_design | Hardware Specifications]]

**Instructions to Set Up a Driver Station:**
1) Download the FRC Game Tools at [this link](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#369633).
	 - Activate the NI product(s) with the serial activation code *B05P01183*.
	 - This will also install the RoboRio Configuration Utility.
2) Download and extract the FRC Radio Configuration Utility with [this link](https://firstfrc.blob.core.windows.net/frc2020/Radio/FRC_Radio_Configuration_20_0_0.zip).
3) Install WPILib at [this link](https://github.com/wpilibsuite/allwpilib/releases/tag/v2021.2.2), downloading Microsoft Visual Studio Code in the process.
4) Download the REV SparkMax Client with [this link](https://www.revrobotics.com/content/sw/rev-hw-client/REV-Hardware-Client-Setup-1.1.0.exe).
5) Download the Phoenix Tuner at [this link](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v5.19.4.1).
6) Download SourceTree with [this link](https://product-downloads.atlassian.com/software/sourcetree/windows/ga/SourceTreeSetup-3.4.3.exe).
7) Download the Limelight Finder at [this link](https://limelightvision.io/pages/downloads).
8) In Visual Studio Code, run the command *WPILib: Manage Vendor Libraries*. Choose the option *Install new libraries (online)* and enter [the Spark Max Vendor Dependencies Link](https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json).
	 - Do the above process again for [the ADIS16470 IMU Vendor Dependencies Link](http://maven.highcurrent.io/vendordeps/ADIS16470.json).

**Robot Specifics:**
 - NI Activation Code: B05P01183
 - Robot Wifi Password: 8114WPAKEY
