// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Motor with simple voltage control. */
public class MotorSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the motor subsystem. */
  public static class Hardware {
    CANSparkMax motor;
    RelativeEncoder encoder;

    public Hardware(CANSparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private double voltageCommand = 0;

  /** Create a new motorSubsystem controlled by a Profiled PID COntroller . */
  public MotorSubsystem(Hardware motorHardware) {
    this.motor = motorHardware.motor;
    this.encoder = motorHardware.encoder;

    initializeMotor();
  }

  private void initializeMotor() {

    motor.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    motor.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    motor.setIdleMode(IdleMode.kBrake);
    DataLogManager.log("Motor firmware version:" + motor.getFirmwareString());
  }

  /**
   * Create hardware devices for the motor subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax motor = new CANSparkMax(Constants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

    return new Hardware(motor, encoder);
  }

  /** Set motor voltage. */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    voltageCommand = voltage;
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    if (enableBrake) {
      DataLogManager.log("Motor set to brake mode");
      motor.setIdleMode(IdleMode.kBrake);
    } else {
      DataLogManager.log("Motor set to coast mode");
      motor.setIdleMode(IdleMode.kCoast);
    }
  }

  /** Returns the motor speed for PID control and logging (Units are RPM). */
  public double getSpeed() {
    return encoder.getVelocity();
  }

  /** Returns the motor commanded voltage. */
  public double getVoltage() {
    return voltageCommand;
  }

  /** Returns the motor current. */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
