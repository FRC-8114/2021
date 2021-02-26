package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IncTrajectoryStateIndex extends CommandBase {
    private RobotContainer robotContainer;

    public IncTrajectoryStateIndex(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void initialize() {
        robotContainer.incIndex();
    }

    public boolean isFinished() {
        return true;
    }
}
