package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SetHoodPosition extends CommandBase {
    double desired_angle, current_angle, speed, moe;
        
    public SetHoodPosition(double angle, double speed, double moe) {
        desired_angle = angle;
        this.speed = speed;
        this.moe = moe;
    }

    public void initialize() {
        
    }

    public void execute() {
        current_angle = ShooterSubsystem.GetHoodEncoderPosition();

        if (current_angle < desired_angle-moe) {
                ShooterSubsystem.IncreaseHoodPosition(speed);
        } else if (current_angle > desired_angle+moe) {
                ShooterSubsystem.LowerHoodPosition(speed);
        } else {
            ShooterSubsystem.StopHood();
        }
    }

    public boolean isFinished() {
        if (current_angle > desired_angle-moe
            && current_angle < desired_angle+moe)
            return true;
        return false;
    }
}
