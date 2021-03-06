package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
    // Index motor controller
    final static CANSparkMax frontIndexController = new CANSparkMax(IndexConstants.FRONT_INDEX_CONTROLLER_PORT,
            MotorType.kBrushed);
    final static CANSparkMax towerIndexController = new CANSparkMax(IndexConstants.TOWER_INDEX_CONTROLLER_PORT,
            MotorType.kBrushed);

    // Creates the IndexSubsystem
    public IndexSubsystem() {
        frontIndexController.restoreFactoryDefaults();
        frontIndexController.setIdleMode(IdleMode.kBrake);
        frontIndexController.setInverted(true);

        towerIndexController.restoreFactoryDefaults();
        towerIndexController.setIdleMode(IdleMode.kBrake);
    }

    public static void FrontIndexRun(double speed) {
        frontIndexController.set(speed);
    }

    public static void TowerIndexRun(double speed) {
        towerIndexController.set(speed);
    }

    public static void AllIndexRun(double speed) {
        FrontIndexRun(speed);
        TowerIndexRun(speed);
    }

    public static void FrontIndexStop() {
        frontIndexController.stopMotor();
    }

    public static void TowerIndexStop() {
        towerIndexController.stopMotor();
    }

    public static void AllIndexStop() {
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