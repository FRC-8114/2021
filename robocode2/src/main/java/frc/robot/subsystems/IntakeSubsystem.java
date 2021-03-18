package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    //the intake motor controller
    final CANSparkMax intakeController = new CANSparkMax(IntakeConstants.INTAKE_CONTROLLER_PORT, MotorType.kBrushless);

    //the intake motor controller encoder
    final CANEncoder intakeControllerEncoder = intakeController.getEncoder();

    //Creates the IntakeSubsystem
    public IntakeSubsystem() {

        //set to factory default and idle so we know what we're working with 
        intakeController.restoreFactoryDefaults(); 
        intakeController.setIdleMode(IdleMode.kBrake);

        //encoders?

    }

    public void IntakeRun(double speed) {
        intakeController.set(speed);
    }

    public void IntakeReverse(double speed) {
        intakeController.set(-speed);
    }

    public void IntakeStop() {
        intakeController.stopMotor();
    }


}