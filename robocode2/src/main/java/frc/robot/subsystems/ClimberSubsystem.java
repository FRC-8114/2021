package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase {
    final CANSparkMax climberController = new CANSparkMax(21, MotorType.kBrushed);

    //Creates the IntakeSubsystem
    public ClimberSubsystem() {

        //set to factory default and idle so we know what we're working with 
        climberController.restoreFactoryDefaults();
        climberController.set(0);
    }

    public void ClimberUp(double speed) {
        climberController.set(speed);
    }

    public void ClimberDown(double speed) {
        climberController.set(-speed);
    }

    public void ClimberStop() {
        climberController.set(0);
    }
}