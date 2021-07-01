// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonFX driveAFollower = new WPI_TalonFX(11);
  private final WPI_TalonFX driveALeader = new WPI_TalonFX(12);
  private final WPI_TalonFX driveBFollower = new WPI_TalonFX(13);
  private final WPI_TalonFX driveBLeader = new WPI_TalonFX(14);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
