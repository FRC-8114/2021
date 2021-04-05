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
    }

    public void initialize() {
        System.out.println("Initialize");
    }

    public void execute() {
        System.out.println("X Offset = " + limelightSubsystem.getTargetXAngle());
        if (limelightSubsystem.getTargetXAngle() < -LimelightConstants.AUTO_CENTER_TOLERANCE) {  
            System.out.println("I DO THE DRIVE");
            driveSystem.tankDrive(-.1, .1);
        } else if (limelightSubsystem.getTargetXAngle() > LimelightConstants.AUTO_CENTER_TOLERANCE) {
            System.out.println("I DRIVE THE DO");
            driveSystem.tankDrive(.1, -.1);
        } else {
            System.out.println("I print therefore I am");
            driveSystem.tankDrive(0, 0);
        }
    }

    public boolean isFinished() {
        if (limelightSubsystem.getTargetXAngle() < LimelightConstants.AUTO_CENTER_TOLERANCE
            && limelightSubsystem.getTargetXAngle() > -LimelightConstants.AUTO_CENTER_TOLERANCE) {
            System.out.println("IS FINISH");
            return true;
        }
        System.out.println("IS SWEEDISH");
        return false;
    }
}
