// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX driveLeftFollower = new WPI_TalonFX(11);
  private final WPI_TalonFX driveLeftLeader = new WPI_TalonFX(12);
  private final WPI_TalonFX driveRightFollower = new WPI_TalonFX(13);
  private final WPI_TalonFX driveRightLeader = new WPI_TalonFX(14);

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
    driveRightFollower.setInverted(true);
    driveRightLeader.setInverted(true);

    driveLeftFollower.follow(driveLeftLeader);
    driveRightFollower.follow(driveRightLeader);

    driveLeftLeader.setSensorPhase(false);
    driveRightLeader.setSensorPhase(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
