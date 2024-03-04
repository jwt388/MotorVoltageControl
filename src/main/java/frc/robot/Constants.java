// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  // Set to true to send telemetry data to Live Window. To false
  // to disable it.
  public static final boolean LW_TELEMETRY_ENABLE = false;

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to log each frame of command execution. To false to disable.
  public static final boolean COMMAND_EXECUTE_LOG = false;

  /** Constants used for the Motor subsystem. */
  public static final int MOTOR_PORT = 2;

  public static final double MOTOR_FIXED_VOLTAGE = 3.0;
  public static final double MOTOR_MAX_JOYSTICK_VOLTAGE = 6.0;
  public static final double DEADBAND = 0.05;

  /** Constants used for assigning operator input. */
  public static final int OPERATOR_CONTROLLER_PORT = 0;
}
