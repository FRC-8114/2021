package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.shooterSubsystem.*;

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

    private double current_angle = 0;
    private double startingVelocity = 20.5; //meters per second
    public static double angle = 0;

    //PIDController hoodPid = new PIDController(0,0,0);

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

        //hoodPid.setTolerance(0.5);

        Shuffleboard.getTab("Shooting").add("Reset Hood Angle", false)
            .withWidget(BuiltInWidgets.kCommand).getEntry()
            .addListener(event -> {
                HoodZero();
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Shuffleboard.getTab("Driving").add("HoodPID P", 1)
        //     .withWidget(BuiltInWidgets.kNumberSlider).getEntry()
        //     .addListener(event -> {
        //         hoodPid.setP(event.value.getDouble());
        //     }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Shuffleboard.getTab("Driving").add("HoodPID I", 0)
        //     .withWidget(BuiltInWidgets.kNumberSlider).getEntry()
        //     .addListener(event -> {
        //         hoodPid.setI(event.value.getDouble());
        //     }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Shuffleboard.getTab("Driving").add("HoodPID D", 0)
        //     .withWidget(BuiltInWidgets.kNumberSlider).getEntry()
        //     .addListener(event -> {
        //         hoodPid.setD(event.value.getDouble());
        //     }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public void periodic() {
        current_angle = hoodControllerEncoder.getPosition();
        SmartDashboard.putNumber("encoderDegrees", hoodControllerEncoder.getPosition());
        SmartDashboard.putNumber("hoodAngle", current_angle);
        SmartDashboard.putNumber("desiredAngle", angle);
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

    public double GetHoodEncoderPosition() {
        return hoodControllerEncoder.getPosition();
    }

    // public double CalculateHoodPID(double measurement, double setpoint) {
    //     return hoodPid.calculate(measurement,setpoint);
    // }

    public double InchesToMeters(double inches) {
        return inches / 39.37;
    }

    public void SetHoodPosition(double degrees) {
        current_angle = hoodControllerEncoder.getPosition();

        // hoodController.set(CalculateHoodPID(current_angle, degrees));
        // SmartDashboard.putNumber("pidCalculate", CalculateHoodPID(current_angle, degrees));

        if (current_angle <= degrees-ShooterConstants.DEGREE_TOLERANCE ||
               current_angle >= degrees+ShooterConstants.DEGREE_TOLERANCE)
        {
            if (current_angle <= degrees-ShooterConstants.DEGREE_TOLERANCE)
                hoodController.set(.05);
            else if (current_angle >= degrees+ShooterConstants.DEGREE_TOLERANCE)
                hoodController.set(-.05);
        }
        else
            hoodController.set(0);
    }

    

    public double CalculateAutoAngle(double x, double y) {
        double g = 9.81;
        x = InchesToMeters(x);
        y = InchesToMeters(y);

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

    public double calculateDesiredVelocity() {
        
        return 0;
    }
}