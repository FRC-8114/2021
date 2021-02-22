package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends CommandBase {
    private DriveSubsystem driveSystem;

    public ResetOdometry(DriveSubsystem driveSystem) {
        this.driveSystem = driveSystem;
    }

    public void initialize() {
        driveSystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    public boolean isFinished() {
        return true;
    }
}
