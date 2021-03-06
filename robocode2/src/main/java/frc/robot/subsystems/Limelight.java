package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class Limelight extends SubsystemBase {
  private NetworkTable limelightTable;
  private static NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry thor;
  private static NetworkTableEntry ledMode;
  private final double TARGET_HEIGHT = 90, LIMELIGHT_HEIGHT = 35, HEIGHT_DIFFERENCE = TARGET_HEIGHT - LIMELIGHT_HEIGHT; // Measurements
                                                                                                                        // in
                                                                                                                        // inches
  private final double LIMELIGHT_HORIZONTAL_ANGLE = 20; // Measured in degrees
  private final double TRIGONOMIC_WEIGHT = 0;
  public double trigDist = 0;
  public static double autoCenterTolerance;
  public static double shooterLimelightOffsetAngle;

  /**
   * Initalizes the Networktable pulling for the Limelight subsystem on the
   * RoboRio sets initial values if there are any present on the table
   */
  public Limelight(String limelightTableName) {
    /*
     * The following creates a variable in which the table for the Limelight is
     * stored and then stores the following as entries: the targets x angle*, the
     * targets y angle, the percentage of the screen the target covers, the status
     * of the limelight's LEDs
     * 
     * *The limelight stores x and y offset from center based off of angle in the
     * camera's FOV (x -27<->27; y -20.5<->20.5)
     */

    limelightTable = NetworkTableInstance.getDefault().getTable(limelightTableName); // The instance is not given a
                                                                                     // variable all limelights are on
                                                                                     // the same instance

    autoCenterTolerance = LimelightConstants.AUTO_CENTER_TOLERANCE;
    shooterLimelightOffsetAngle = LimelightConstants.SHOOTER_LIMELIGHT_OFFSET_ANGLE;

    Shuffleboard.getTab("Limelight").add("Auto Center Tolerance", LimelightConstants.AUTO_CENTER_TOLERANCE)
        .withWidget(BuiltInWidgets.kNumberSlider).getEntry().addListener(event -> {
          autoCenterTolerance = event.value.getDouble();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    Shuffleboard.getTab("Limelight")
        .add("Shooter-Limelight Offset Angle", LimelightConstants.SHOOTER_LIMELIGHT_OFFSET_ANGLE)
        .withWidget(BuiltInWidgets.kNumberSlider).getEntry().addListener(event -> {
          shooterLimelightOffsetAngle = event.value.getDouble();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // The following variales are named after the values they access as opposed to
    // what said values represent
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    thor = limelightTable.getEntry("thor");
    ledMode = limelightTable.getEntry("ledMode");
  }

  /*
   * Runs operations that should occur
   */
  public void periodic() {
    // SmartDashboard.putNumber("Distance to Target", approximateDistance());
    trigDist = trigonomicDistance();
    SmartDashboard.putNumber("Limelight Trigonometric Distance", trigDist);
    SmartDashboard.putNumber("Width Distance", widthDistance());
    SmartDashboard.putNumber("Area Distance", areaDistance());
    SmartDashboard.putNumber("Limelight Angle", LIMELIGHT_HORIZONTAL_ANGLE + getTargetYAngle());
    SmartDashboard.putNumber("Height Difference", HEIGHT_DIFFERENCE);

  }

  /*
   * Returns the x angle of the target relative to the center of the camera's view
   * 
   * @return The angle in degrees
   */
  public static double getTargetXAngle() {
    return tx.getDouble(0.0);
  }

  public double getTargetWidth() {
    return thor.getDouble(0.0);
  }

  /*
   * Returns the y angle of the target relative to the center of the camera's view
   * 
   * @return The angle in degrees
   */
  public double getTargetYAngle() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  /*
   * Returns the percentage of the limelight's view the target covers
   * 
   * @return The percentage in decimal form
   */
  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  /*
   * Returns the mode of the Limelight's LEDs 1 = force off, 2 = force blink, 3 =
   * force on
   * 
   * @return The LEDs mode
   */
  public double getLEDMode() {
    return ledMode.getDouble(0.0);
  }

  /*
   * Changes the LED mode of the Limelight 1 = force off, 2 = force blink, 3 =
   * force on
   * 
   * @param mode The double representing the desired mode
   */
  public static void setLEDMode(double mode) {
    ledMode.forceSetNumber(mode);
  }

  // Sets LEDmode to 1, forcing the limelight to turn off
  public static void turnOffLED() {
    setLEDMode(1);
  }

  // Sets LEDmode to 3, forcing the limelight to turn on
  public static void turnOnLED() {
    setLEDMode(3);
  }

  /*
   * Calculates a weighted average between the two methods of calculating the robot's distance from the target
   * 
   * @return    The weighted average distance
   */
  public double approximateDistance() {
    return (widthDistance() + areaDistance()) / 2;
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

  public double widthDistance() {
    return 536 + -9.07*getTargetWidth() + 0.0474 * (Math.pow(getTargetWidth(), 2));
  }

  public double areaDistance() {
    return 209*(Math.pow(getTargetArea(), -.564));
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