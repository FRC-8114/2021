package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
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
    }

    public void initialize() {
        System.out.println("Initialize");
    }

    public void execute() {
        System.out.println("X Offset = " + limelightSubsystem.getTargetXAngle());
        if (limelightSubsystem.getTargetXAngle() < -LimelightConstants.AUTO_CENTER_TOLERANCE)    
            driveSystem.tankDrive(-.1, .1);
        else if (limelightSubsystem.getTargetXAngle() > LimelightConstants.AUTO_CENTER_TOLERANCE)
            driveSystem.tankDrive(.1, -.1);
        else
            driveSystem.tankDrive(0, 0);
    }

    public boolean isFinished() {
        if (limelightSubsystem.getTargetXAngle() < LimelightConstants.AUTO_CENTER_TOLERANCE
            && limelightSubsystem.getTargetXAngle() > -LimelightConstants.AUTO_CENTER_TOLERANCE)
            return true;
        return false;
    }
}
