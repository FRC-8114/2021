package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.controller.PIDController;
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
    final CANSparkMax hoodController = new CANSparkMax(ShooterConstants.HOOD_CONTROLLER_PORT, MotorType.kBrushless);

    // Shooter motor controller encoders
    final CANEncoder leftShooterControllerEncoder = leftShooterController.getEncoder();
    final CANEncoder rightShooterControllerEncoder = rightShooterController.getEncoder();
    final CANEncoder kickerControllerEncoder = kickerController.getEncoder();
    final CANEncoder hoodControllerEncoder = hoodController.getEncoder();

    private double current_angle = 0;

    PIDController shooterPid = new PIDController(0,0,0);

    // Creates the ShooterSubsystem
    public ShooterSubsystem() {
        leftShooterController.restoreFactoryDefaults(); 
        leftShooterController.setIdleMode(IdleMode.kCoast);
        leftShooterController.setInverted(true);

        rightShooterController.restoreFactoryDefaults(); 
        rightShooterController.setIdleMode(IdleMode.kCoast);

        kickerController.restoreFactoryDefaults();
        kickerController.setIdleMode(IdleMode.kBrake);
        kickerController.setInverted(true);

        hoodController.restoreFactoryDefaults();
        hoodController.setIdleMode(IdleMode.kBrake);

        leftShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);
        leftShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        rightShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);
        rightShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        hoodControllerEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);
        hoodControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
        HoodZero();

        shooterPid.setTolerance(0.5);
        shooterPid.setSetpoint(calculateDesiredVelocity());

        Shuffleboard.getTab("Shooting").add("Reset Hood Angle", false)
            .withWidget(BuiltInWidgets.kCommand).getEntry()
            .addListener(event -> {
                HoodZero();
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void periodic() {
        SmartDashboard.putNumber("encoderDegrees", hoodControllerEncoder.getPosition());
        SmartDashboard.putNumber("hoodAngle", current_angle);   
    }

    public void ShooterRun(double speed) {
        leftShooterController.set(speed);
        rightShooterController.set(speed);
    }

    public void KickerRun(double speed) {
        kickerController.set(speed);
    }

    public void ShooterStop() {
        leftShooterController.stopMotor();
        rightShooterController.stopMotor();
    }

    public void KickerStop() {
        kickerController.stopMotor();
    }

    public void ShooterReverse(double speed) {
        leftShooterController.set(-speed);
        rightShooterController.set(-speed);
    }

    public void KickerReverse(double speed) {
        kickerController.set(-speed);
    }

    public void IncreaseHoodPosition(double speed) {
        hoodController.set(speed);
    }

    public void StopHood() {
        hoodController.stopMotor();
    }

    public void LowerHoodPosition(double speed) {
        hoodController.set(-speed);
    }

    public void HoodZero() {
        hoodControllerEncoder.setPosition(0);
    }

    public void SetHoodPosition(double degrees) {
        current_angle = hoodControllerEncoder.getPosition();

        if (current_angle <= degrees-ShooterConstants.DEGREE_TOLERANCE ||
               current_angle >= degrees+ShooterConstants.DEGREE_TOLERANCE)
        {
            if (current_angle <= degrees-ShooterConstants.DEGREE_TOLERANCE)
                hoodController.set(0.1);
            else if (current_angle >= degrees+ShooterConstants.DEGREE_TOLERANCE)
                hoodController.set(-0.1);
        }
    }

    

    public void AutoShoot(double speed) {
        leftShooterController.set(shooterPid.calculate(leftShooterControllerEncoder.getVelocity()));
    }

    public double calculateDesiredVelocity() {
        
        return 0;
    }
}