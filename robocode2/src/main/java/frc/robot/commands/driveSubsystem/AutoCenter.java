package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        double offset = limelightSubsystem.getTargetXAngle() + limelightSubsystem.shooterLimelightOffsetAngle;
        if (offset < -limelightSubsystem.autoCenterTolerance) {  
            driveSystem.cheesyDrive(0, -0.1, true);
        } else if (offset > limelightSubsystem.autoCenterTolerance) {
            driveSystem.cheesyDrive(0, 0.1, true);
        } else {
            driveSystem.cheesyDrive(0, 0, true);
        }
    }

    public boolean isFinished() {
        double offset = limelightSubsystem.getTargetXAngle() + limelightSubsystem.shooterLimelightOffsetAngle;
        if (offset < limelightSubsystem.autoCenterTolerance && 
            offset > -limelightSubsystem.autoCenterTolerance) {
            return true;
        }
        return false;
    }
}
