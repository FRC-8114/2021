package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SetHoodPosition extends CommandBase {
    double desired_angle, current_angle;
        
    public SetHoodPosition(double angle) {
        desired_angle = angle;
    }

    public void initialize() {
        
    }

    public void execute() {
        current_angle = ShooterSubsystem.GetHoodEncoderPosition();

        if (current_angle < desired_angle-ShooterConstants.HOOD_DEGREE_TOLERANCE) {
                ShooterSubsystem.IncreaseHoodPosition(0.1);
        } else if (current_angle > desired_angle+ShooterConstants.HOOD_DEGREE_TOLERANCE) {
                ShooterSubsystem.LowerHoodPosition(0.1);
        } else {
            ShooterSubsystem.StopHood();
        }
    }

    public boolean isFinished() {
        if (current_angle > desired_angle-ShooterConstants.HOOD_DEGREE_TOLERANCE
            && current_angle < desired_angle+ShooterConstants.HOOD_DEGREE_TOLERANCE)
            return true;
        return false;
    }
}
