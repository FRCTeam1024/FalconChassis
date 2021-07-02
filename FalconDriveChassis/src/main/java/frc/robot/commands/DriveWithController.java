// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.Drivetrain;

public class DriveWithController extends CommandBase {
  private final Drivetrain drivetrain;
  private final Logitech controller;

  /** Creates a new DriveWithController. */
  public DriveWithController(Drivetrain driveSubsystem, Logitech controllerParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);
    controller = controllerParam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(-controller.getLeftStickY() / 4, -controller.getRightStickY() / 4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}