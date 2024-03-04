// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MotorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // First we do things that are in all Robots.
  private PowerDistribution pdp = new PowerDistribution();

  // The operator's controller
  private CommandXboxController operatorController =
      new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // Now the subsystems.
  private final MotorSubsystem motor = new MotorSubsystem(MotorSubsystem.initializeHardware());

  private double fixedVoltage = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    ShuffleboardTab sbMotorTab = Shuffleboard.getTab("Motor");

    sbMotorTab
        .add(
            new InstantCommand(() -> motor.setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Brake Mode"))
        .withSize(3, 1)
        .withPosition(0, 0);

    sbMotorTab
        .add(
            new InstantCommand(() -> motor.setBrakeMode(false))
                .ignoringDisable(true)
                .withName("Coast Mode"))
        .withSize(3, 1)
        .withPosition(0, 1);

    sbMotorTab
        .addNumber("Motor Voltage", motor::getVoltage)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(3, 1)
        .withPosition(3, 0);

    sbMotorTab
        .addNumber("Motor Speed", motor::getSpeed)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(3, 1)
        .withPosition(3, 1);

    sbMotorTab
        .addNumber("Motor Current", motor::getCurrent)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(3, 1)
        .withPosition(3, 2);

    SmartDashboard.putNumber("Trigger Voltage", Constants.MOTOR_FIXED_VOLTAGE);
    SmartDashboard.putNumber("Max Joystick Voltage", Constants.MOTOR_MAX_JOYSTICK_VOLTAGE);

    // Set the default motor command
    motor.setDefaultCommand(getMotorCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Run the motor forward at the defined speed while the right trigger is held.
    operatorController
        .rightTrigger()
        .whileTrue(
            new RunCommand(() -> motor.setVoltage(fixedVoltage), motor)
                .finallyDo(() -> motor.setVoltage(0.0))
                .withName("Motor: Run Forward"));
    // Run the motor in reverse at the defined speed while the left trigger is held.
    operatorController
        .leftTrigger()
        .whileTrue(
            new RunCommand(() -> motor.setVoltage(-fixedVoltage), motor)
                .finallyDo(() -> motor.setVoltage(0.0))
                .withName("Motor: Run Reverse"));
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getMotorCommand() {

    fixedVoltage = SmartDashboard.getNumber("Trigger Voltage", Constants.MOTOR_FIXED_VOLTAGE);
    double maxJoystickVoltage =
        SmartDashboard.getNumber("Max Joystick Voltage", Constants.MOTOR_MAX_JOYSTICK_VOLTAGE);

    return new RunCommand(
            () ->
                motor.setVoltage(
                    -maxJoystickVoltage
                        * MathUtil.applyDeadband(
                            operatorController.getLeftY(), Constants.DEADBAND)),
            motor)
        .withName("Joystick Command");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  /**
   * Use this to get the PDP for data logging.
   *
   * @return The PowerDistribution module.
   */
  public PowerDistribution getPdp() {
    return this.pdp;
  }

  /**
   * Use this to get the Launcher Subsystem.
   *
   * @return a reference to the Launcher Subsystem
   */
  public MotorSubsystem getMotorSubsystem() {
    return motor;
  }
}
