package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoCenter extends CommandBase {
    private DriveSubsystem driveSystem;
    private Limelight limelightSubsystem;

    public AutoCenter(DriveSubsystem driveSystem, Limelight limelightSubsystem) {
        this.driveSystem = driveSystem;
        this.limelightSubsystem = limelightSubsystem;
        System.out.println("Constructor");

        m_requirements.add(driveSystem);
        m_requirements.add(limelightSubsystem);
    }

    public void initialize() {
        System.out.println("Initialize");
    }

    public void execute() {
        System.out.println("X Offset = " + limelightSubsystem.getTargetXAngle());
        if (limelightSubsystem.getTargetXAngle() < -3.5 -LimelightConstants.AUTO_CENTER_TOLERANCE) {  
            driveSystem.cheesyDrive(0, -.1, true);
        } else if (limelightSubsystem.getTargetXAngle() > -3.5 + LimelightConstants.AUTO_CENTER_TOLERANCE) {
            driveSystem.cheesyDrive(0, +.1, true);
        } else {
            driveSystem.tankDrive(0, 0);
        }
    }

    public boolean isFinished() {
        if (limelightSubsystem.getTargetXAngle() < -3.5 + LimelightConstants.AUTO_CENTER_TOLERANCE
            && limelightSubsystem.getTargetXAngle() > -3.5 -LimelightConstants.AUTO_CENTER_TOLERANCE) {
            return true;
        }
        return false;
    }
}
