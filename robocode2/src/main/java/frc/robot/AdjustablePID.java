package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class AdjustablePID extends TimedRobot {
  public CANPIDController pidController;
  public CANSparkMax motor;
  public CANEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, setpoint;
  public String tabName;
  public boolean inControl = false, mode = false;

  public AdjustablePID(CANSparkMax motor, String tabName) {
    // initialize motor
    this.motor = motor;
    this.tabName = tabName;

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    pidController = motor.getPIDController();
    encoder = motor.getEncoder();

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    Shuffleboard.getTab(tabName).add("P Gain", kP)
        .getEntry().addListener(event -> {
            kP = event.value.getDouble();
        }, EntryListenerFlags.kUpdate);
    Shuffleboard.getTab(tabName).add("I Gain", kI)
        .getEntry().addListener(event -> {
            kI = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);
    Shuffleboard.getTab(tabName).add("D Gain", kD)
        .getEntry().addListener(event -> {
            kD = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);
    Shuffleboard.getTab(tabName).add("I Zone", kIz)
        .getEntry().addListener(event -> {
            kIz = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);
    Shuffleboard.getTab(tabName).add("Feed Forward", kFF)
        .getEntry().addListener(event -> {
            kFF = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);
    Shuffleboard.getTab(tabName).add("Max Output", kMaxOutput)
        .getEntry().addListener(event -> {
            kMaxOutput = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);
    Shuffleboard.getTab(tabName).add("Min Output", kMinOutput)
        .getEntry().addListener(event -> {
            kMinOutput = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);

    // display Smart Motion coefficients
    Shuffleboard.getTab(tabName).add("Max Velocity", maxVel)
        .getEntry().addListener(event -> {
            maxVel = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);;
    Shuffleboard.getTab(tabName).add("Min Velocity", minVel)
        .getEntry().addListener(event -> {
            minVel = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);;
    Shuffleboard.getTab(tabName).add("Max Acceleration", maxAcc)
        .getEntry().addListener(event -> {
            maxAcc = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);;
    Shuffleboard.getTab(tabName).add("Allowed Closed Loop Error", allowedErr)
        .getEntry().addListener(event -> {
            allowedErr = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);;
    
    
    // Setpoint and mode editing
    Shuffleboard.getTab(tabName).add("Setpoint", 0)
        .getEntry().addListener(event -> {
            setpoint = event.value.getDouble();
    }, EntryListenerFlags.kUpdate);;
    Shuffleboard.getTab(tabName).add("Mode", false)
        .getEntry().addListener(event -> {
            mode = event.value.getBoolean();
        }, EntryListenerFlags.kUpdate);
    Shuffleboard.getTab(tabName).add("In Control", false)
        .withWidget(BuiltInWidgets.kToggleButton).getEntry().addListener(event -> {
            inControl = event.value.getBoolean();
        }, EntryListenerFlags.kUpdate);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }

  @Override
  public void teleopPeriodic() {
    if(inControl) {
        pidController.setP(kP);
        pidController.setP(kI);
        pidController.setP(kD);
    } else {
        pidController.setP(0);
        pidController.setP(0);
        pidController.setP(0);
    }

    double processVariable;
    if(mode) {
      pidController.setReference(setpoint, ControlType.kVelocity);
      processVariable = encoder.getVelocity();
    } else {
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      pidController.setReference(setpoint, ControlType.kSmartMotion);
      processVariable = encoder.getPosition();
    }
    
    Shuffleboard.getTab(tabName).add("Process Variable", processVariable);
    Shuffleboard.getTab(tabName).add("Output", motor.getAppliedOutput());
  }
}