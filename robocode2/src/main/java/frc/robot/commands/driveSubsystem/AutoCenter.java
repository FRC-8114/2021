package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoCenter extends CommandBase {
    private DriveSubsystem DriveSubsystem;

    public AutoCenter() {
        System.out.println("Constructor");
    }

    public void initialize() {
        System.out.println("Initialize");
        Limelight.turnOnLED();
    }

    public void execute() {
        double offset = Limelight.getTargetXAngle() + Limelight.shooterLimelightOffsetAngle;
        if (offset < -Limelight.autoCenterTolerance) {  
            DriveSubsystem.cheesyDrive(0, -0.1, true);
        } else if (offset > Limelight.autoCenterTolerance) {
            DriveSubsystem.cheesyDrive(0, 0.1, true);
        } else {
            DriveSubsystem.cheesyDrive(0, 0, true);
        }
    }

    public boolean isFinished() {
        double offset = Limelight.getTargetXAngle() + Limelight.shooterLimelightOffsetAngle;
        if (offset < Limelight.autoCenterTolerance && 
            offset > -Limelight.autoCenterTolerance) {
            return true;
        }
        return false;
    }

    public void end(boolean interrupted) {
        Limelight.turnOffLED();
    }
}
