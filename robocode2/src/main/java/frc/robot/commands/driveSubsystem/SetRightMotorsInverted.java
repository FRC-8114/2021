package frc.robot.commands.driveSubsystem;

import javax.management.modelmbean.RequiredModelMBean;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetRightMotorsInverted extends CommandBase {
    private DriveSubsystem driveSystem;
    private boolean inverted;

    public SetRightMotorsInverted(DriveSubsystem driveSystem, boolean inverted) {
        this.driveSystem = driveSystem;
        this.inverted = inverted;

        m_requirements.add(driveSystem);
    }

    public void initialize() {
        driveSystem.setRightMotorsInverted(inverted);
    }

    public boolean isFinished() {
        return true;
    }
}
