package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoCenter extends CommandBase {
    private DriveSubsystem DriveSubsystem;
    private double speed, offsetAngle, moe;

    public AutoCenter(double speed, double offsetAngle, double moe) {
        this.speed = speed;
        this.offsetAngle = offsetAngle;
        this.moe = moe;
    }

    public void initialize() {
        Limelight.turnOnLED();
    }

    public void execute() {
        double offset = Limelight.getTargetXAngle() + offsetAngle;
        if (offset < -moe) {  
            DriveSubsystem.cheesyDrive(0, -speed, true);
        } else if (offset > moe) {
            DriveSubsystem.cheesyDrive(0, speed, true);
        } else {
            DriveSubsystem.cheesyDrive(0, 0, true);
        }
    }

    public boolean isFinished() {
        double offsetAngle = Limelight.getTargetXAngle() + Limelight.shooterLimelightOffsetAngle;
        if (offsetAngle < Limelight.autoCenterTolerance && 
            offsetAngle > -Limelight.autoCenterTolerance) {
            return true;
        } else if(!DriveSubsystem.canAutoCenter) {
            return true;
        }
        return false;
    }

    public void end(boolean interrupted) {
        Limelight.turnOffLED();
    }
}
