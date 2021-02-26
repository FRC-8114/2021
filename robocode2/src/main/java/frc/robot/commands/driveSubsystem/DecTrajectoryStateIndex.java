package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DecTrajectoryStateIndex extends CommandBase {
    private RobotContainer robotContainer;

    public DecTrajectoryStateIndex(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void initialize() {
        robotContainer.decIndex();
    }

    public boolean isFinished() {
        return true;
    }
}
