// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;
import java.time.Instant;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;

public class DrivetrainSubsystem extends SubsystemBase {
  private CANSparkMax m_frontLeftMotor;
  private CANSparkMax m_frontRightMotor;
  private CANSparkMax m_rearLeftMotor;
  private CANSparkMax m_rearRightMotor;
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    m_frontLeftMotor = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
    m_frontLeftMotor.setInverted(Constants.Drivetrain.kFrontLeftInverted);
    m_frontLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_frontLeftMotor.setIdleMode(IdleMode.kCoast);
    m_frontLeftMotor.burnFlash();

    m_frontRightMotor = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
    m_frontRightMotor.setInverted(Constants.Drivetrain.kFrontRightInverted);
    m_frontRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_frontRightMotor.setIdleMode(IdleMode.kCoast);
    m_frontRightMotor.burnFlash();

    m_rearLeftMotor = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
    m_rearLeftMotor.setInverted(Constants.Drivetrain.kRearLeftInverted);
    m_rearLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_rearLeftMotor.setIdleMode(IdleMode.kCoast);
    m_rearLeftMotor.burnFlash();

    m_rearRightMotor = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
    m_rearRightMotor.setInverted(Constants.Drivetrain.kRearRightInverted);
    m_rearRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_rearRightMotor.setIdleMode(IdleMode.kCoast);
    m_rearRightMotor.burnFlash();

    m_leftEncoder = new Encoder(5, 6);
    m_rightEncoder = new Encoder(1, 2);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the right distance of the two encoders.
   *
   * @return the right of the two encoder readings
   */
  public double getRightEncoderDistance() {
    return m_rightEncoder.getDistance();
  }

  /**
   * Gets the left distance of the two encoders.
   *
   * @return the left of the two encoder readings
   */
  public double getLeftEncoderDistance() {
    return m_leftEncoder.getDistance();
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void driveArcade(double _straight, double _turn) {
    double left = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
    double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);

    m_frontLeftMotor.set(left);
    m_frontRightMotor.set(right);
    m_rearLeftMotor.set(left);
    m_rearRightMotor.set(right);
  }

  public void tankDrive(double _left, double _right) {
    m_frontLeftMotor.set(_left);
    m_frontRightMotor.set(_right);
    m_rearLeftMotor.set(_left);
    m_rearRightMotor.set(_right);
  }

  public void autoMode(int duration) throws InterruptedException {
    long stopTime = Instant.now().getEpochSecond() + duration;

    while (stopTime > Instant.now().getEpochSecond()) {
      driveArcade(0.4, 0);
    }
    driveArcade(0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint =
    // val);
    // builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    // addChild("Controller", m_controller);
  }
}
