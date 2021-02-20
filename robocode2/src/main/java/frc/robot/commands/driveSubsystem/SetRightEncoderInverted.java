package frc.robot.commands.driveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetRightEncoderInverted extends CommandBase {
    private DriveSubsystem driveSystem;
    private boolean inverted;

    public SetRightEncoderInverted(DriveSubsystem driveSystem, boolean inverted) {
        this.driveSystem = driveSystem;
        this.inverted = inverted;

        m_requirements.add(driveSystem);
    }

    public void initialize() {
        driveSystem.setRightEncoderInverted(inverted);
    }

    public boolean isFinished() {
        return true;
    }
}
