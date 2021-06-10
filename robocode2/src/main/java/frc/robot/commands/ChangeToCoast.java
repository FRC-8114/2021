package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChangeToCoast extends CommandBase {
    double time;
    Timer timer = new Timer();

    public ChangeToCoast(double seconds) {
        time = seconds;
    }

    public void initialize() {
        timer.start();
    }

    public void execute() {

    }

    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();

        DriveSubsystem.leftMotorLeader.setIdleMode(IdleMode.kCoast);
        DriveSubsystem.leftMotorFollower.setIdleMode(IdleMode.kCoast);
        DriveSubsystem.rightMotorLeader.setIdleMode(IdleMode.kCoast);
        DriveSubsystem.rightMotorFollower.setIdleMode(IdleMode.kCoast);
    }

    public boolean isFinished() {
        if (timer.get() > time)
            return true;
        return false;
    }
}
