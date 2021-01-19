package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.XboxController;
public class DriveSubsystem extends SubsystemBase {
  //the xbox controller, which is initialized in the Robot.java file
  private XboxController controller;
  
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors;

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors;

  // The robot's drive
  private final DifferentialDrive m_drive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    //assigns the left motor controllers to this speed controller group    
    m_leftMotors =
      new SpeedControllerGroup(
          new CANSparkMax(1, MotorType.kBrushless),
          new CANSparkMax(2, MotorType.kBrushless));  
    
    //assigns the right motor controllers to this speed controller group
    m_rightMotors =
      new SpeedControllerGroup(
        new CANSparkMax(3, MotorType.kBrushless),
        new CANSparkMax(4, MotorType.kBrushless)); 

    //creates a differential drive object
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    
  }

  public void drive(double left, double right) {
    //pass in two values for the left and right motor speeds
    m_drive.tankDrive(left, right, false);
  }
}