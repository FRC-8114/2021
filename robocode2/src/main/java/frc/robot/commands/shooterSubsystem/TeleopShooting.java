package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.driveSubsystem.AutoCenter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class TeleopShooting extends CommandBase {
    XboxController controller;
    double rpm;

    public TeleopShooting(XboxController controller, double rpm) {
        this.controller = controller;
        this.rpm = rpm;
    }

    public void initialize() {
        new AutoCenter().schedule();
        new ShooterRun(rpm).schedule();
    }

    public void execute() {
        double shooterRPM = SmartDashboard.getNumber("Flywheel Process Variable", 0);
        if (shooterRPM >= rpm-25 /* && shooterRPM <= 3675 */) {
            IndexSubsystem.AllIndexRun(0.6);
            KickerSubsystem.KickerRun(0.8);
        }

        else if (shooterRPM < rpm-35) {
            IndexSubsystem.AllIndexStop();
            KickerSubsystem.KickerStop();
        }
    }

    public void end(boolean interrupted) {
        new StopShooter().schedule();
        IndexSubsystem.AllIndexStop();
        KickerSubsystem.KickerStop();
    }

    public boolean isFinished() {
        if (controller.getTriggerAxis(Hand.kRight) != 1)
            return true;
        return false;
    }
}