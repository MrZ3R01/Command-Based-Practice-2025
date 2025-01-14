// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;

public class RobotContainer {
  private Arm arm = new Arm();
  private Drivetrain drivetrain = new Drivetrain();
  private Intake intake = new Intake();
  private CommandXboxController controller = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    arm.resetEncoderCommand();
    controller.rightBumper().whileTrue(intake.intakeBag());
    controller.leftBumper().whileTrue(intake.outtakeBag());
    controller.rightBumper().or(controller.leftBumper()).onFalse(intake.stopIntakeM());
    controller.x().whileTrue(arm.resetEncoderCommand());
    drivetrain.setDefaultCommand(
      drivetrain.arcadeDrive(()->controller.getLeftY(), ()->controller.getRightX())
    );

    arm.setDefaultCommand(
      arm.setArmMovementCommand()
    );

    controller.a().onTrue(arm.moveArmToNewGoal(Constants.Arm.PID.setpoints.HIGH));
    controller.b().onTrue(arm.moveArmToNewGoal(Constants.Arm.PID.setpoints.GROUND));
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      arm.moveArmToNewGoal(Constants.Arm.PID.setpoints.GROUND),
      drivetrain.arcadeDrive(()->0.2, ()->0.0).withTimeout(3));
  }
}
