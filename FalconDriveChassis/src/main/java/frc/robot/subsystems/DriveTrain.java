// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonFX driveLeftFollower = new WPI_TalonFX(Constants.DriveConstants.driveLeftFollowerID);
  private final WPI_TalonFX driveLeftLeader = new WPI_TalonFX(Constants.DriveConstants.driveLeftLeaderID);
  private final WPI_TalonFX driveRightFollower = new WPI_TalonFX(Constants.DriveConstants.driverRightFollowerID);
  private final WPI_TalonFX driveRightLeader = new WPI_TalonFX(Constants.DriveConstants.driveRightLeaderID);

  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(driveRightLeader, driveRightFollower);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(driveLeftLeader, driveLeftFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final AHRS navX;  //Gyro may not be navX, some of this may need to be redone

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveTrain. */
  public Drivetrain() {
    // Reset to default
    driveLeftFollower.configFactoryDefault();
    driveLeftLeader.configFactoryDefault();
    driveRightFollower.configFactoryDefault();
    driveRightLeader.configFactoryDefault();

    driveLeftFollower.setNeutralMode(NeutralMode.Brake);
    driveLeftLeader.setNeutralMode(NeutralMode.Brake);
    driveRightFollower.setNeutralMode(NeutralMode.Brake);
    driveRightLeader.setNeutralMode(NeutralMode.Brake);

    driveLeftFollower.setInverted(false);
    driveLeftLeader.setInverted(false);
    driveRightFollower.setInverted(true); //these were set to true for driving. When testing with the trajectory code, they're having issues when set to true.
    driveRightLeader.setInverted(true);

    driveLeftFollower.follow(driveLeftLeader);
    driveRightFollower.follow(driveRightLeader);

    driveLeftLeader.setSensorPhase(false);
    driveRightLeader.setSensorPhase(false);

    AHRS a = null;
    try{
      a = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }
    navX = a;
    navX.reset();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(navX.getRotation2d(), 
        (driveLeftLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation),
        (-1 * driveRightLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation));
    m_drive.feedWatchdog();
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

   /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Returns the speed on both sides of the drivetrain, as well as reporting to SmartDahsboard
   * 
   * @return Speed of both sides of the drivetrain
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    SmartDashboard.putNumber("Raw Left Encoder", driveLeftLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Raw Right Encoder", 1 * driveRightLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("Gyro Angle", getHeading());
    return new DifferentialDriveWheelSpeeds(10 * driveLeftLeader.getSelectedSensorVelocity() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation, 
                                            10 * driveRightLeader.getSelectedSensorVelocity() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation);
  }

  /**
   * @return Average distance of the two sides of the drivetrain
   */

  public double getAverageEncoderDistance() {
    return ((driveLeftLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation) 
            + -1 * driveRightLeader.getSelectedSensorPosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation) / 2.0;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    navX.reset();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navX.getRotation2d());
  }

  public void resetEncoders() {
    driveLeftLeader.setSelectedSensorPosition(0);
    driveRightLeader.setSelectedSensorPosition(0);
  }

  /* 
   * Drive command for use with joystick drive and debugging purpose
   * Do not use for auto routines as this will not be repeatable
   */
  public void drive(double leftPower, double rightPower) {
    driveLeftLeader.set(ControlMode.PercentOutput, leftPower);
    driveRightLeader.set(ControlMode.PercentOutput, rightPower);
  }
}
