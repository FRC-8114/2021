package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter motor controllers
    final CANSparkMax leftShooterController = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    final CANSparkMax rightShooterController = new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    final CANSparkMax kickerController = new CANSparkMax(ShooterConstants.KICKER_CONTROLLER_PORT, MotorType.kBrushless);
    public final CANSparkMax hoodController = new CANSparkMax(ShooterConstants.HOOD_CONTROLLER_PORT, MotorType.kBrushless);

    // Shooter motor controller encoders
    final CANEncoder leftShooterControllerEncoder = leftShooterController.getEncoder();
    final CANEncoder rightShooterControllerEncoder = rightShooterController.getEncoder();
    final CANEncoder kickerControllerEncoder = kickerController.getEncoder();
    final CANEncoder hoodControllerEncoder = hoodController.getEncoder();

    private double current_angle = 0, startingVelocity = 20.5;
    public double angle = 0;

    // Creates the ShooterSubsystem
    public ShooterSubsystem() {
        leftShooterController.restoreFactoryDefaults(); 
        leftShooterController.setIdleMode(IdleMode.kCoast);
        leftShooterController.setInverted(true);

        rightShooterController.restoreFactoryDefaults(); 
        rightShooterController.setIdleMode(IdleMode.kCoast);
        rightShooterController.follow(leftShooterController, true);

        kickerController.restoreFactoryDefaults();
        kickerController.setIdleMode(IdleMode.kBrake);
        kickerController.setInverted(true);

        hoodController.restoreFactoryDefaults();
        hoodController.setIdleMode(IdleMode.kBrake);

        leftShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        leftShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        rightShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.SHOOTER_DISTANCE_PER_PULSE);
        rightShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        hoodControllerEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);
        hoodControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
        HoodZero();

        Shuffleboard.getTab("Reset Hood Angle Party").add("Reset Hood Angle", false)
            .withWidget(BuiltInWidgets.kCommand).getEntry()
            .addListener(event -> {
                HoodZero();
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        Shuffleboard.getTab("Shooting").add("Shooter Control", ShooterConstants.MAX_INPUT)
            .withWidget(BuiltInWidgets.kNumberSlider).getEntry()
            .addListener(event -> {
              ShooterConstants.MAX_INPUT = event.value.getDouble();
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void periodic() {
        current_angle = hoodControllerEncoder.getPosition();
        SmartDashboard.putNumber("encoderDegrees", hoodControllerEncoder.getPosition());
        SmartDashboard.putNumber("hoodAngle", current_angle);
        SmartDashboard.putNumber("desiredAngle", angle);
    }

    public double verifyVelocity(double speed) {
        int sign = (int) (speed / Math.abs(speed));
        if (Math.abs(speed) > ShooterConstants.MAX_INPUT)
            return sign * ShooterConstants.MAX_INPUT;
        return speed;
    }

    public void ShooterRun(double speed) {
        leftShooterController.set(verifyVelocity(speed));
    }

    public void KickerRun(double speed) {
        kickerController.set(verifyVelocity(speed));
    }

    public void ShooterStop() {
        leftShooterController.stopMotor();
    }

    public void KickerStop() {
        kickerController.stopMotor();
    }

    public void ShooterReverse(double speed) {
        leftShooterController.set(-verifyVelocity(speed));
    }

    public void KickerReverse(double speed) {
        kickerController.set(-verifyVelocity(speed));
    }

    public void IncreaseHoodPosition(double speed) {
        hoodController.set(verifyVelocity(speed));
    }

    public void StopHood() {
        hoodController.stopMotor();
    }

    public void LowerHoodPosition(double speed) {
        hoodController.set(-verifyVelocity(speed));
    }

    public void HoodZero() {
        hoodControllerEncoder.setPosition(7.3456);
    }

    public double GetHoodEncoderPosition() {
        return hoodControllerEncoder.getPosition();
    }

    public double InchesToMeters(double inches) {
        return inches / 39.37;
    }

    public double CalculateAutoAngle(double x, double y) {
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

    /**
     * Calculates desired velocity given angle, x, y
     * Should prioritize changing velocity over changing angle when possible
     * If return is greater than max velocity then change to max velocity and run find angle
     * @param angle
     * @param x
     * @param y
     * @return desired velocity
     */
    public double calculateDesiredVelocity(double angle, double x, double y) {
        // double desired_velocity =
        // (tanangle * g * x)^2 = v^4 +- v^4 - g^2 * x^2 - 2*g*y*v^2
        // (tanangle * g * x)^2 + g^2 * x^2 = v^4 +- v^4 - 2*g*y*v^2
        // ((tanangle * g * x)^2 + g^2 * x^2) / 2*g*y = v^4 +- v^4 - v^2
        return 0;
    }
}