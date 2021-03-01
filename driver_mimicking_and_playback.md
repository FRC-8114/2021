Data will be recorded with one file for value (ie left_motor_input.txt, right_motor_input.txt, left_encoder_pos.txt) with each entry seperated by a comma for splicing. Datafiles for the same recording session will be stored in a common directory.
	
Both mimicking and playback will be decentralized processes, with each subsystem contianing the necessary methods to do both when a global variable is altered.

To prevent the overwriting of data when new code is deployed, the recordings will be saved on the RIO outside of the robot project. This method removes the need to have a networktable system setup to allow the driverstation to save values during recording and to redeploy to test a recording.

Two global variables will be used for playback and mimicking: a string for the directory name and an on/off boolean (might use a single "status" string). The string for mimicking will be directly edited over networktables while the string for playback will be altered through a SendableChooser.

Example Structure
resources
  a_red
  	left_motor_input.txt
	right_motor_input.txt
	left_encoder_pos.txt
	left_encoder_rate.txt
	...
  a_blue
  	...
  b_red
  	...
  b_blue
  	...