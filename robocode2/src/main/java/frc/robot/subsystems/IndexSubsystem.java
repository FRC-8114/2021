package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
    //the index motor controller
    final CANSparkMax indexController = new CANSparkMax(IndexConstants.INDEX_CONTROLLER_PORT, MotorType.kBrushless);

    //the index motor controller encoder
    final CANEncoder indexControllerEncoder = indexController.getEncoder();

    //Creates the IndexSubsystem
    public IndexSubsystem() {

        //set to factory default and idle so we know what we're working with 
        indexController.restoreFactoryDefaults(); 
        indexController.setIdleMode(IdleMode.kBrake);

    }

    public void IndexRun(double speed) {
        indexController.set(speed);
    }

    public void IndexReverse(double speed) {
        indexController.set(-speed);
    }

    public void IndexStop() {
        indexController.stopMotor();
    }


}