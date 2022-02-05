// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class DriveForwardCommand extends CommandBase {

  DriveTrain driveTrain;
  double startPoint;

  /** Creates a new DriveForwardCommand. */
  public DriveForwardCommand(DriveTrain d) {
    driveTrain = d;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPoint = driveTrain.getDistanceInInches();
    System.out.println("Drive Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(0.7, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentPosition;
    currentPosition = driveTrain.getDistanceInInches();
    return Math.abs(startPoint - currentPosition) > 40;
  }
}
