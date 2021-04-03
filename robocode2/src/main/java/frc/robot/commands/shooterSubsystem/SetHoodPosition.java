package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SetHoodPosition extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    double desired_angle, current_angle;
    
    public SetHoodPosition(ShooterSubsystem shooterSubsystem, double angle) {
        this.shooterSubsystem = shooterSubsystem;
        desired_angle = angle;
    }

    public void initialize() {
        
    }

    public void execute() {
        current_angle = shooterSubsystem.GetHoodEncoderPosition();

        if (current_angle < desired_angle-ShooterConstants.DEGREE_TOLERANCE ||
               current_angle > desired_angle+ShooterConstants.DEGREE_TOLERANCE)
        {
            if (current_angle < desired_angle-ShooterConstants.DEGREE_TOLERANCE)
                shooterSubsystem.hoodController.set(.1);
            else if (current_angle > desired_angle+ShooterConstants.DEGREE_TOLERANCE)
                shooterSubsystem.hoodController.set(-.1);
        }
        else
            shooterSubsystem.hoodController.set(0);
    }

    public boolean isFinished() {
        if (current_angle > desired_angle-ShooterConstants.DEGREE_TOLERANCE
            && current_angle < desired_angle+ShooterConstants.DEGREE_TOLERANCE)
            return true;
        return false;
    }
}
