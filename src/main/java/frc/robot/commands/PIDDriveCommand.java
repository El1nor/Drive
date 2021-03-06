// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import pabeles.concurrency.IntOperatorTask.Max;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDDriveCommand extends PIDCommand {
  private DriveTrain driveTrain;
 
  /** Creates a new PiddyDriveCommand. */
  public PIDDriveCommand(DriveTrain d) {
    super(
        // The controller that the command will use
        new PIDController(0.0075, 0, 0.0001),
        // This should return the measurement
        () -> d.getDistanceInInches(),
        // This should return the setpoint (can also be a constant)
        () -> 120,
        // This uses the output
        output -> {
          // clamp the output of the PID to -1 and 1
          d.drive(d.clamp(output), d.clamp(output));
          // Use the output here
        },
        d);
    driveTrain = d;

    getController().setTolerance(0.1, 0.1);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  
}
