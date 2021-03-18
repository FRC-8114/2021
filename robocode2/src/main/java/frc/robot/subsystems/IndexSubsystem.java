package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
    // Index motor controller
    final PWMVictorSPX frontIndexController = new PWMVictorSPX(IndexConstants.FRONT_INDEX_CONTROLLER_PORT);
    final CANSparkMax towerIndexController = new CANSparkMax(IndexConstants.TOWER_INDEX_CONTROLLER_PORT, MotorType.kBrushless);

    final CANEncoder towerIndexControllEncoder = towerIndexController.getEncoder();

    // Creates the IndexSubsystem
    public IndexSubsystem() {
        towerIndexController.restoreFactoryDefaults();
        towerIndexController.setIdleMode(IdleMode.kBrake);
    }

    public void FrontIndexRun (double speed) {
        frontIndexController.set(speed);
    }

    public void TowerIndexRun (double speed) {
        towerIndexController.set(speed);
    }

    public void AllIndexRun (double speed) {
        FrontIndexRun(speed);
        TowerIndexRun(speed);
    }

    public void FrontIndexStop() {
        frontIndexController.stopMotor();
    }

    public void TowerIndexStop() {
        towerIndexController.stopMotor();
    }

    public void AllIndexStop() {
        FrontIndexStop();
        TowerIndexStop();
    }

    public void FrontIndexReverse(double speed) {
        frontIndexController.set(-speed);
    }

    public void TowerIndexReverse(double speed) {
        towerIndexController.set(-speed);
    }

    public void AllIndexReverse(double speed) {
        FrontIndexReverse(speed);
        TowerIndexReverse(speed);
    }
}