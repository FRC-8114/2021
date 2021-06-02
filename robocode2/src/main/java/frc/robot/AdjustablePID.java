package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
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
public class AdjustablePID {
  public static CANPIDController pidController;
  public CANSparkMax motor;
  public CANEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, setpoint;
  public String pidName;
  public boolean inControl = false, mode = false;
  public NetworkTableEntry processVariableEntry, outputEntry, actualP;

  public AdjustablePID(CANSparkMax motor, String pidName) {
    // initialize motor
    this.motor = motor;
    this.pidName = pidName;

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
    SmartDashboard.putNumber(pidName + " P Gain", kP);
    SmartDashboard.putNumber(pidName + " I Gain", kI);
    SmartDashboard.putNumber(pidName + " D Gain", kD);
    SmartDashboard.putNumber(pidName + " I Zone", kIz);
    SmartDashboard.putNumber(pidName + " Feed Forward", kFF);
    SmartDashboard.putNumber(pidName + " Max Output", kMaxOutput);
    SmartDashboard.putNumber(pidName + " Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber(pidName + " Max Velocity", maxVel);
    SmartDashboard.putNumber(pidName + " Min Velocity", minVel);
    SmartDashboard.putNumber(pidName + " Max Acceleration", maxAcc);
    SmartDashboard.putNumber(pidName + " Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber(pidName + " Set Position", 0);
    SmartDashboard.putNumber(pidName + " Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean(pidName + " Mode", true);
  }

  public AdjustablePID(CANSparkMax motor, String pidName, double kP, double kI, double kD, double kIz, double kFF) {
    // initialize motor
    this.motor = motor;
    this.pidName = pidName;

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
    this.kP = kP; 
    this.kI = kI;
    this.kD = kD; 
    this.kIz = kIz; 
    this.kFF = kFF; 
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
    SmartDashboard.putNumber(pidName + " P Gain", kP);
    SmartDashboard.putNumber(pidName + " I Gain", kI);
    SmartDashboard.putNumber(pidName + " D Gain", kD);
    SmartDashboard.putNumber(pidName + " I Zone", kIz);
    SmartDashboard.putNumber(pidName + " Feed Forward", kFF);
    SmartDashboard.putNumber(pidName + " Max Output", kMaxOutput);
    SmartDashboard.putNumber(pidName + " Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber(pidName + " Max Velocity", maxVel);
    SmartDashboard.putNumber(pidName + " Min Velocity", minVel);
    SmartDashboard.putNumber(pidName + " Max Acceleration", maxAcc);
    SmartDashboard.putNumber(pidName + " Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber(pidName + " Set Position", 0);
    SmartDashboard.putNumber(pidName + " Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean(pidName + " Mode", true);
  }

  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber(pidName + " P Gain", 0);
    double i = SmartDashboard.getNumber(pidName + " I Gain", 0);
    double d = SmartDashboard.getNumber(pidName + " D Gain", 0);
    double iz = SmartDashboard.getNumber(pidName + " I Zone", 0);
    double ff = SmartDashboard.getNumber(pidName + " Feed Forward", 0);
    double max = SmartDashboard.getNumber(pidName + " Max Output", 0);
    double min = SmartDashboard.getNumber(pidName + " Min Output", 0);
    double maxV = SmartDashboard.getNumber(pidName + " Max Velocity", 0);
    double minV = SmartDashboard.getNumber(pidName + " Min Velocity", 0);
    double maxA = SmartDashboard.getNumber(pidName + " Max Acceleration", 0);
    double allE = SmartDashboard.getNumber(pidName + " Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean(pidName + " Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber(pidName + " Set Velocity", 0);
      pidController.setReference(setPoint, ControlType.kVelocity);
      processVariable = encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber(pidName + " Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      pidController.setReference(setPoint, ControlType.kSmartMotion);
      processVariable = encoder.getPosition();
    }
    
    SmartDashboard.putNumber(pidName + " SetPoint", setPoint);
    SmartDashboard.putNumber(pidName + " Process Variable", processVariable);
    SmartDashboard.putNumber(pidName + " Output", motor.getAppliedOutput());
  }
}