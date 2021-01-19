package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class LimelightSubsystem extends SubsystemBase {
  private NetworkTable limelightTable;
  private NetworkTableEntry tx, ty, ta, ledMode;
  private final double TARGET_HEIGHT = 81.0625, LIMELIGHT_HEIGHT = 8.5, HEIGHT_DIFFERENCE = TARGET_HEIGHT - LIMELIGHT_HEIGHT; // Measurements in inches
  private final double LIMELIGHT_HORIZONTAL_ANGLE = 34.324911622; // Measured in degrees
  private final double TRIGONOMIC_WEIGHT = 0; 
  private final DriveSubsystem m_drive = new DriveSubsystem();

  /**
   * Initalizes the Networktable pulling for the Limelight subsystem on the RoboRio
   * sets initial values if there are any present on the table
   */
  public LimelightSubsystem(String limelightTableName) {
    /*
     * The following creates a variable in which the table for the Limelight is stored
     * and then stores the following as entries: the targets x angle*, the targets y
     * angle, the percentage of the screen the target covers, the status of the limelight's
     * LEDs
     * 
     * *The limelight stores x and y offset from center based off of angle in the camera's
     * FOV (x -27<->27; y -20.5<->20.5)
     */

    limelightTable = NetworkTableInstance.getDefault().getTable(limelightTableName); // The instance is not given a variable all limelights are on the same instance
    
    // The following variales are named after the values they access as opposed to what said values represent
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    ledMode = limelightTable.getEntry("ledMode");
  }

  /*
   * Runs operations that should occur
   */
  public void periodic() {
    SmartDashboard.putNumber("Distance to Target", approximateDistance());


  }

  public void findTarget() {
      for (double i = getTargetXAngle(); i > 0.1 || i < -0.1; i = getTargetXAngle()) {
          if (i < -0.1)
            m_drive.drive(.25,-.25);
          else if (i > 0.1)
            m_drive.drive(-.25, .25);
          else
            m_drive.drive(0,0);
      }
  }

  /*
   * Returns the x angle of the target relative to the center of the camera's
   * view
   * 
   * @return    The angle in degrees
   */
  public double getTargetXAngle() {
    return tx.getDouble(0.0);
  }

  /*
   * Returns the y angle of the target relative to the center of the camera's
   * view
   * 
   * @return    The angle in degrees
   */
  public double getTargetYAngle() {
    return ty.getDouble(0.0);
  }

  /*
   * Returns the percentage of the limelight's view the target covers
   * 
   * @return    The percentage in decimal form
   */
  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  /*
   * Returns the mode of the Limelight's LEDs
   * 1 = force off, 2 = force blink, 3 = force on
   * 
   * @return    The LEDs mode
   */
  public double getLEDMode() {
    return ledMode.getDouble(0.0);
  }

  /*
   * Changes the LED mode of the Limelight
   * 1 = force off, 2 = force blink, 3 = force on
   * 
   * @param mode    The double representing the desired mode
   */
  public void setLEDMode(double mode) {
    ledMode.setNumber(mode);
  }

  /*
   * Calculates a weighted average between the two methods of calculating the robot's distance from the target
   * 
   * @return    The weighted average distance
   */
  public double approximateDistance() {
    return TRIGONOMIC_WEIGHT*trigonomicDistance() + (1-TRIGONOMIC_WEIGHT)*dataDrivenDistance();
  }

  /*
   * Calculates the approximate distance form the target using its relative angle to the camera's horizontal
   * 
   * @return    The approximate distance to target in inches
   */
  public double trigonomicDistance() {
    double angleToHorizontal = LIMELIGHT_HORIZONTAL_ANGLE + getTargetYAngle();
    return HEIGHT_DIFFERENCE / Math.tan(angleToHorizontal);
  }

  /*
   * Calculates the target's approximate distance from the robot through a formula calculated using Area:Distance datapoints
   * 
   * @return    The approximate distance to the target in inches
   */
  public double dataDrivenDistance() {
    return (-99.9 * Math.log(getTargetArea())) + 162;
  }
}