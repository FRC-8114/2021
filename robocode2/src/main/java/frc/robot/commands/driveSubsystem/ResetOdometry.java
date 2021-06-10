package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends CommandBase {
    public ResetOdometry() {
    }

    public void initialize() {
        DriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    public boolean isFinished() {
        return true;
    }
}
