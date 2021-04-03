package frc.robot.commands.shooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SetAutoAngle extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    double angle = 0;
    double g = 9.81;
    double x = 0;
    double y = 0;
    
    public SetAutoAngle(ShooterSubsystem shooterSubsystem, double distance, double height) {
        this.shooterSubsystem = shooterSubsystem;
        this.x = shooterSubsystem.InchesToMeters(distance);
        this.y = shooterSubsystem.InchesToMeters(height);
    }

    public void initialize() {
        double startingVelocity = ShooterConstants.BALL_VELOCITY;

        angle = Math.toDegrees(Math.atan((Math.pow(startingVelocity, 2)
                + Math.sqrt(Math.pow(startingVelocity, 4) - g * (g * Math.pow(x, 2)) + 2 * y * Math.pow(startingVelocity, 2)))
                / (g * x)));

        if (angle < 0)
            angle = Math.toDegrees(Math.atan((Math.pow(startingVelocity, 2)
                - Math.sqrt(Math.pow(startingVelocity, 4) - g * (g * Math.pow(x, 2)) + 2 * y * Math.pow(startingVelocity, 2)))
                / (g * x)));

        angle = 45 - (90 - angle) / 2;
        
        SmartDashboard.putNumber("desiredAngle", angle);

        new SetHoodPosition(shooterSubsystem, angle);
    }

    public void execute() {
        
    }

    public boolean isFinished() {
        if (shooterSubsystem.GetHoodEncoderPosition() > angle-ShooterConstants.DEGREE_TOLERANCE
            && shooterSubsystem.GetHoodEncoderPosition() < angle+ShooterConstants.DEGREE_TOLERANCE)
            return true;
        return false;
    }
}
