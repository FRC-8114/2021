package edu.wpi.first.wpilibj.examples.ramsetecommand.subsystems;
package com.revrobotics;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final SpeedControllerGroup m_leftMotors = 
        new SpeedControllerGroup(new CANSparkMax(DriveConstants.kLeftMotor1Port, CANSparkMaxLowLevel.MotorType.kBrushless),
                                 new PWMSparkMax(DriveConstants.kLeftMotor2Port));
    
    private final SpeedControllerGroup m_rightMotors = 
        new SpeedControllerGroup(new PWMSparkMax(DriveConstants.kLeftMotor1Port),
                                 new PWMSparkMax(DriveConstants.kLeftMotor2Port));
    
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    private final 
}