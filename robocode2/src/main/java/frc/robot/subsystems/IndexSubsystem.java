package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
    // Index motor controller
    final CANSparkMax indexController = new CANSparkMax(IndexConstants.INDEX_CONTROLLER_PORT, MotorType.kBrushless);

    // Index motor controller encoder
    final CANEncoder indexControllerEncoder = indexController.getEncoder();

    // Creates the IndexSubsystem
    public IndexSubsystem() {
        indexController.restoreFactoryDefaults(); 
        indexController.setIdleMode(IdleMode.kBrake);

    }

    public void IndexRun(double speed) {
        indexController.set(speed);
    }

    public void IndexStop() {
        indexController.stopMotor();
    }
    
    public void IndexReverse(double speed) {
        indexController.set(-speed);
    }
}