package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase {
    final TalonFX leftClimberController = new TalonFX(60);
    //final TalonFX rightClimberController = new TalonFX(1);

    //the intake motor controller encoder
    //final CANEncoder leftClimberControllerEncoder = leftClimberController.g;
    //final CANEncoder rightClimberControllerEncoder = rightClimberController;

    //Creates the IntakeSubsystem
    public ClimberSubsystem() {

        //set to factory default and idle so we know what we're working with 
        leftClimberController.configFactoryDefault(); 
        leftClimberController.set(ControlMode.PercentOutput, 0);


        // rightClimberController.configFactoryDefault();
        // rightClimberController.set(ControlMode.PercentOutput, 0);
    }

    public void ClimberUp(double speed) {
        leftClimberController.set(ControlMode.PercentOutput, speed);
    }

    public void ClimberDown(double speed) {
        leftClimberController.set(ControlMode.PercentOutput, -speed);
    }

    public void ClimberStop() {
        leftClimberController.set(ControlMode.PercentOutput, 0);
    }
}