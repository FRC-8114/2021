package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class KickerSubsystem extends SubsystemBase {
    public final CANSparkMax kickerController = new CANSparkMax(ShooterConstants.KICKER_CONTROLLER_PORT, MotorType.kBrushless);
    final CANEncoder kickerControllerEncoder = kickerController.getEncoder();

    public KickerSubsystem() {
        kickerController.restoreFactoryDefaults();
        kickerController.setIdleMode(IdleMode.kBrake);
        kickerController.setInverted(true);
    }

    public void KickerRun(double speed) {
        kickerController.set(speed);
    }


    public void KickerStop() {
        kickerController.stopMotor();
    }


    public void KickerReverse(double speed) {
        kickerController.set(-speed);
    }

}
