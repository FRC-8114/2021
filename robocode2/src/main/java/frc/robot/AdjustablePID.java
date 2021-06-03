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
  public CANPIDController pidController;
  public CANSparkMax motor;
  public CANEncoder encoder;
  public String pidName;
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

    // set PID coefficients
    pidController.setP(5e-5);
    pidController.setI(1e-6);
    pidController.setD(0);
    pidController.setIZone(0);
    pidController.setFF(0.000156);
    pidController.setOutputRange(-1, 1);

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
    pidController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber(pidName + " P Gain", 5e-5);
    SmartDashboard.putNumber(pidName + " I Gain", 1e-6);
    SmartDashboard.putNumber(pidName + " D Gain", 0);
    SmartDashboard.putNumber(pidName + " I Zone", 0);
    SmartDashboard.putNumber(pidName + " Feed Forward", 0.000156);
    SmartDashboard.putNumber(pidName + " Max Output", 1);
    SmartDashboard.putNumber(pidName + " Min Output", -1);

    // display Smart Motion coefficients
    SmartDashboard.putNumber(pidName + " Max Velocity", 2000);
    SmartDashboard.putNumber(pidName + " Min Velocity", 0);
    SmartDashboard.putNumber(pidName + " Max Acceleration", 1500);
    SmartDashboard.putNumber(pidName + " Allowed Closed Loop Error", 0);
    SmartDashboard.putNumber(pidName + " Set Position", 0);
    SmartDashboard.putNumber(pidName + " Set Velocity", 0);
    SmartDashboard.putNumber(pidName + " SetPoint", 0);

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

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(-1, 1);

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
    pidController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(1500, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber(pidName + " P Gain", kP);
    SmartDashboard.putNumber(pidName + " I Gain", kI);
    SmartDashboard.putNumber(pidName + " D Gain", kD);
    SmartDashboard.putNumber(pidName + " I Zone", kIz);
    SmartDashboard.putNumber(pidName + " Feed Forward", kFF);
    SmartDashboard.putNumber(pidName + " Max Output", 1);
    SmartDashboard.putNumber(pidName + " Min Output", -1);

    // display Smart Motion coefficients
    SmartDashboard.putNumber(pidName + " Max Velocity", 2000);
    SmartDashboard.putNumber(pidName + " Min Velocity", 0);
    SmartDashboard.putNumber(pidName + " Max Acceleration", 1500);
    SmartDashboard.putNumber(pidName + " Allowed Closed Loop Error", 0);
    SmartDashboard.putNumber(pidName + " Set Position", 0);
    SmartDashboard.putNumber(pidName + " Set Velocity", 0);
    SmartDashboard.putNumber(pidName + " SetPoint", 0);

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

    double kP = pidController.getP();
    double kI = pidController.getI();
    double kD = pidController.getD();
    double kIz = pidController.getIZone();
    double kFF = pidController.getFF();
    double kMaxOutput = pidController.getOutputMin();
    double kMinOutput = pidController.getOutputMax();
    double maxVel = pidController.getSmartMotionMaxVelocity(0);
    double minVel = pidController.getSmartMotionMinOutputVelocity(0);
    double maxAcc = pidController.getSmartMotionMaxAccel(0);
    double allowedErr = pidController.getSmartMotionAllowedClosedLoopError(0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) pidController.setP(p);
    if((i != kI)) pidController.setI(i); 
    if((d != kD)) pidController.setD(d);
    if((iz != kIz)) pidController.setIZone(iz);
    if((ff != kFF)) pidController.setFF(ff);
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
    }
    if((maxV != maxVel)) pidController.setSmartMotionMaxVelocity(maxV,0);
    if((minV != minVel)) pidController.setSmartMotionMinOutputVelocity(minV,0);
    if((maxA != maxAcc)) pidController.setSmartMotionMaxAccel(maxA,0);
    if((allE != allowedErr)) pidController.setSmartMotionAllowedClosedLoopError(allE,0);

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

    SmartDashboard.putNumber(pidName + " PID P", pidController.getP());
    SmartDashboard.putNumber(pidName + " PID I", pidController.getI());
    SmartDashboard.putNumber(pidName + " PID D", pidController.getD());
    SmartDashboard.putNumber(pidName + " PID Iz", pidController.getIZone());
    SmartDashboard.putNumber(pidName + " PID FF", pidController.getFF());
  }
}