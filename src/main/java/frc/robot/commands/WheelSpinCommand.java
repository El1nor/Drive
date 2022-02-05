// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class WheelSpinCommand extends CommandBase {

  DriveTrain driveTrain;
  double startPoint;
  /** Creates a new WheelSpinCommand. */
  public WheelSpinCommand(DriveTrain d) {
    driveTrain = d;
    addRequirements(d);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPoint = driveTrain.getDistanceInInches();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(0,-0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition;
    currentPosition = driveTrain.getDistanceInInches();
    return Math.abs(startPoint - currentPosition) > 21;
  }
}
