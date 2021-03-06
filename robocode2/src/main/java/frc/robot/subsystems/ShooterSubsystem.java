package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.security.KeyStore.Entry;

import com.revrobotics.CANEncoder;

import frc.robot.AdjustablePID;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter motor controllers
    public final static CANSparkMax leftShooterController = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    final CANSparkMax rightShooterController = new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    //public final CANSparkMax kickerController = new CANSparkMax(ShooterConstants.KICKER_CONTROLLER_PORT, MotorType.kBrushless);
    public final static CANSparkMax hoodController = new CANSparkMax(ShooterConstants.HOOD_CONTROLLER_PORT,
            MotorType.kBrushless);

    // Shooter motor controller encoders
    final CANEncoder leftShooterControllerEncoder = leftShooterController.getEncoder();
    final CANEncoder rightShooterControllerEncoder = rightShooterController.getEncoder();
    //final CANEncoder kickerControllerEncoder = kickerController.getEncoder();
    final static CANEncoder hoodControllerEncoder = hoodController.getEncoder();
	public static final double ShooterRPM = 0;

    // PIDs
    public final static AdjustablePID flywheelPID = new AdjustablePID(leftShooterController, "Flywheel", 0.00005, 0.0000001, 0, 0, 0.000156);

    public final static NetworkTableEntry encoderDegreesEntry = Shuffleboard.getTab("Robot Control").add("Hood/Hood Angle", 0).getEntry();

    private double current_angle = 0;
    public double angle = 0, velocity = 0;
    public static double speed = leftShooterController.getAppliedOutput();

    // Creates the ShooterSubsystem
    public ShooterSubsystem() {
        leftShooterController.restoreFactoryDefaults();
        leftShooterController.setIdleMode(IdleMode.kCoast);
        leftShooterController.setInverted(true);

        rightShooterController.restoreFactoryDefaults();
        rightShooterController.setIdleMode(IdleMode.kCoast);
        rightShooterController.follow(leftShooterController, true);

        hoodController.restoreFactoryDefaults();
        hoodController.setIdleMode(IdleMode.kBrake);

        leftShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        leftShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        rightShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        rightShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        hoodControllerEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);
        hoodControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
        HoodZero();

        Shuffleboard.getTab("Reset Hood Angle Party").add("Reset Hood Angle", false).withWidget(BuiltInWidgets.kCommand)
                .getEntry().addListener(event -> {
                    HoodZero();
                }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        Shuffleboard.getTab("Shooting").add("Shooter Control", ShooterConstants.MAX_INPUT)
                .withWidget(BuiltInWidgets.kNumberSlider).getEntry().addListener(event -> {
                    ShooterConstants.MAX_INPUT = event.value.getDouble();
                }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
    }

    public void periodic() {
        current_angle = hoodControllerEncoder.getPosition();
        SmartDashboard.putNumber("encoderDegrees", hoodControllerEncoder.getPosition());
        SmartDashboard.putNumber("hoodAngle", current_angle);
        SmartDashboard.putNumber("desiredAngle", angle);
        SmartDashboard.putNumber("desiredVelocity", velocity);
        SmartDashboard.putNumber("actualVelocity", speed);

        encoderDegreesEntry.forceSetDouble(hoodControllerEncoder.getPosition());

        flywheelPID.periodic();
    }

    public static double verifyVelocity(double speed) {
        int sign = (int) (speed / Math.abs(speed));
        if (Math.abs(speed) > ShooterConstants.MAX_INPUT)
            return sign * ShooterConstants.MAX_INPUT;
        return speed;
    }

    public static void ShooterRun(double rpm) {
        SmartDashboard.putNumber("Flywheel Set Velocity", rpm);
    }

    public static void ShooterStop() {
        SmartDashboard.putNumber("Flywheel Set Velocity", 0);
    }

    public void ShooterReverse(double rpm) {
        SmartDashboard.putNumber("Flywheel Set Velocity", -rpm);
    }

    // Hood Positions should be one for 0-5 ft in front of goal and one for 9-14 ft from goal
    public static void IncreaseHoodPosition(double speed) {
        hoodController.set(verifyVelocity(speed));
    }

    public static void StopHood() {
        hoodController.stopMotor();
    }

    public static void LowerHoodPosition(double speed) {
        hoodController.set(-verifyVelocity(speed));
    }

    public void HoodZero() {
        hoodControllerEncoder.setPosition(7.3456);
    }

    public static double GetHoodEncoderPosition() {
        return hoodControllerEncoder.getPosition();
    }

    public double InchesToMeters(double inches) {
        return inches / 39.37;
    }

    public double CalculateAutoAngle(double x, double y, double startingVelocity) {
        double g = 9.81;
        x = InchesToMeters(x);
        y = InchesToMeters(y);
        startingVelocity = ShooterConstants.MAX_INPUT * startingVelocity;

        angle = Math.toDegrees(Math.atan((Math.pow(startingVelocity, 2)
                + Math.sqrt(Math.pow(startingVelocity, 4) - g * (g * Math.pow(x, 2)) + 2 * y * Math.pow(startingVelocity, 2)))
                / (g * x)));

        if (angle < 0)
            angle = Math.toDegrees(Math.atan((Math.pow(startingVelocity, 2)
                - Math.sqrt(Math.pow(startingVelocity, 4) - g * (g * Math.pow(x, 2)) + 2 * y * Math.pow(startingVelocity, 2)))
                / (g * x)));

        angle = 45 - (90 - angle) / 2;

        periodic();
        
        return angle;
    }

    public double CalculateAutoVelocity(double x, double y, double angle) {
        double g = 9.81;
        x = InchesToMeters(x);
        y = InchesToMeters(y);
        angle = hoodControllerEncoder.getPosition();

        velocity = Math.sqrt(((g*Math.pow(x,2)) * (Math.pow(Math.tan(angle), 2) + 1) / 
                            (2 * (x * Math.tan(angle) - y))));
        
        if (velocity < 0)
            velocity = -1 * Math.sqrt(((g*Math.pow(x,2)) * (Math.pow(Math.tan(angle), 2) + 1) / 
                        (2 * (x * Math.tan(angle) - y))));

        periodic();

        return velocity;
    }
}