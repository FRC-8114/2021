package frc.robot.commands.driveSubsystem;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.ChangeToCoast;
import frc.robot.subsystems.DriveSubsystem;

public class Backward extends CommandBase{
    private double d, speed;

    public Backward(double d, double speed)
    {
        this.d = d;
        this.speed = -speed;
    }

    public void initialize() {
        DriveSubsystem.tankDrive(speed, speed);
        DriveSubsystem.driverControl = false;

        DriveSubsystem.leftMotorLeader.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftMotorFollower.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightMotorLeader.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightMotorFollower.setIdleMode(IdleMode.kBrake);

    }

    public void execute()
    {
        DriveSubsystem.tankDrive(speed, speed);
    }

    public void end(boolean interrupted)
    {
        DriveSubsystem.tankDrive(0, 0);
        System.out.println(d + "\n" + DriveSubsystem.getAverageEncoderDistance());
        DriveSubsystem.driverControl = true;

        new ChangeToCoast(2).schedule();
    }

    public boolean isFinished()
    {
        if(Math.abs(DriveSubsystem.getAverageEncoderDistance()) < Math.abs(d)) {
            return false;
        }
        return true;
    }
}
