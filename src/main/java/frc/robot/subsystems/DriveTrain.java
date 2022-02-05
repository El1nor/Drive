// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.BoardAxis;

import com.revrobotics.RelativeEncoder;

/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANSparkMax rightFront;
    private CANSparkMax rightBack;
    private MotorControllerGroup rightMotors;
    private CANSparkMax leftBack;
    private CANSparkMax leftFront;
    private MotorControllerGroup leftMotors;
    private DifferentialDrive robotDrive;
    public RelativeEncoder encoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        rightFront = new CANSparkMax(4, MotorType.kBrushless);
        encoder = rightFront.getEncoder();

        rightFront.restoreFactoryDefaults();
        rightFront.setInverted(false);
        rightFront.setIdleMode(IdleMode.kBrake);

        rightBack = new CANSparkMax(5, MotorType.kBrushless);

        rightBack.restoreFactoryDefaults();
        rightBack.setInverted(false);
        rightBack.setIdleMode(IdleMode.kBrake);

        rightMotors = new MotorControllerGroup(rightFront, rightBack);
        addChild("rightMotors", rightMotors);

        leftBack = new CANSparkMax(3, MotorType.kBrushless);

        leftBack.restoreFactoryDefaults();
        leftBack.setInverted(true);
        leftBack.setIdleMode(IdleMode.kBrake);

        leftFront = new CANSparkMax(2, MotorType.kBrushless);

        leftFront.restoreFactoryDefaults();
        leftFront.setInverted(true);
        leftFront.setIdleMode(IdleMode.kBrake);

        leftMotors = new MotorControllerGroup(leftFront, leftBack);
        addChild("leftMotors", leftMotors);

        robotDrive = new DifferentialDrive(leftMotors, rightMotors);
        addChild("robotDrive", robotDrive);
        robotDrive.setSafetyEnabled(true);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(1.0);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder", getDistanceInInches());
        // SmartDashboard.putNumber("angle", navx.getAngle());
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void drive(double lspeed, double rspeed) {
        lspeed *= -1.0;
        rspeed *= -1.0;
        //changes the driving from arcade to tank
        robotDrive.tankDrive(lspeed, rspeed);
    }

    
    public double getAngle() {
        return 0;
        // return navx.getAngle();
    }
    //gets distance in inches
    public double getDistanceInInches() {
        return encoder.getPosition() * -2.1;
    }
    // makes it so the speed can only be between -1 and 1

    public double clamp(double x) {
        return Math.max(-1.0, Math.min(x, 1.0));
      }

}