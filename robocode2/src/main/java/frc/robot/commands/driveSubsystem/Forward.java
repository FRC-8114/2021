package frc.robot.commands.driveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Forward extends CommandBase{
    private double d, speedL, speedR;
    public Forward(double d, double speedL, double speedR)
    {
        this.d = d;
        this.speedL = speedL;
        this.speedR = speedR;
    }

    public void execute()
    {
        DriveSubsystem.tankDrive(speedL, speedR);
    }

    public void end()
    {
        DriveSubsystem.tankDrive(0, 0);
    }

    public boolean isFinished()
    {
        if(DriveSubsystem.getAverageEncoderDistance() < d)
            return false;
        return true;
    }
}
