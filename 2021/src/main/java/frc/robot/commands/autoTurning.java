// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class autoTurning extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private DriveSubsystem m_drive;
    private LimelightSubsystem limelight;
    private double kFindDriveSpeed = Constants.kFindDriveSpeed;
    private double kMaxFindAngle = Constants.kMaxFindAngle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoTurning(LimelightSubsystem limelight, DriveSubsystem m_drive) {
    this.limelight = limelight;
    this.m_drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (double i = limelight.getTargetXAngle(); i > kMaxFindAngle || i < -kMaxFindAngle; i = limelight.getTargetXAngle()) {
      if (i < -kMaxFindAngle)
        m_drive.drive(kFindDriveSpeed,-kFindDriveSpeed);
      else if (i > kMaxFindAngle)
        m_drive.drive(-kFindDriveSpeed, kFindDriveSpeed);
      else
        m_drive.drive(0,0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (-kMaxFindAngle <= limelight.getTargetXAngle() && limelight.getTargetXAngle() <= kMaxFindAngle)
      return true;
    return false;
  }
}
