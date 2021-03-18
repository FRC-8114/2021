package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter motor controller
    final CANSparkMax leftShooterController = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);
    final CANSparkMax rightShooterController = new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_CONTROLLER_PORT, MotorType.kBrushless);

    // Shooter motor controller encoder
    final CANEncoder leftShooterControllerEncoder = leftShooterController.getEncoder();
    final CANEncoder rightShooterControllerEncoder = rightShooterController.getEncoder();

    // Creates the ShooterSubsystem
    public ShooterSubsystem() {
        leftShooterController.restoreFactoryDefaults(); 
        leftShooterController.setIdleMode(IdleMode.kCoast);

        rightShooterController.restoreFactoryDefaults(); 
        rightShooterController.setIdleMode(IdleMode.kCoast);

        leftShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);
        leftShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

        rightShooterControllerEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_DISTANCE_PER_PULSE);
        rightShooterControllerEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

    }

    public void ShooterRun(double speed) {
        leftShooterController.set(speed);
        rightShooterController.set(speed);
    }

    public void ShooterStop() {
        leftShooterController.stopMotor();
        rightShooterController.stopMotor();
    }
}