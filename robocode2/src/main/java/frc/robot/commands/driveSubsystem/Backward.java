package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Backward extends CommandBase {
    private double d, speedL, speedR;
    public Backward(double d, double speedL, double speedR)
    {
        this.d = d/391.5;
        this.speedL = -speedL;
        this.speedR = -speedR;
    }

    public void execute()
    {
        DriveSubsystem.tankDrive(speedL, speedR);
        System.out.println(DriveSubsystem.getAverageEncoderDistance());

    }

    public void end()
    {
        DriveSubsystem.tankDrive(0, 0);
    }

    public boolean isFinished()
    {
        if(Math.abs(DriveSubsystem.getAverageEncoderDistance()) > Math.abs(d)){
            DriveSubsystem.tankDrive(0, 0);
            return true;
        }
            
        return false;
    }
}

