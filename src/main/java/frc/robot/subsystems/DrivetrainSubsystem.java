// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;

public class DrivetrainSubsystem extends SubsystemBase {
    private CANSparkMax m_frontLeftMotor;
    private CANSparkMax m_frontRightMotor;
    private CANSparkMax m_rearLeftMotor;
    private CANSparkMax m_rearRightMotor;

    /** Creates a new DrivetrainSubsystem. */
    public DrivetrainSubsystem() {
        m_frontLeftMotor  = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_frontLeftMotor.setInverted(Constants.Drivetrain.kFrontLeftInverted);
        m_frontLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_frontLeftMotor.setIdleMode(IdleMode.kCoast);
        m_frontLeftMotor.burnFlash();

        m_frontRightMotor = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_frontRightMotor.setInverted(Constants.Drivetrain.kFrontRightInverted);
        m_frontRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_frontRightMotor.setIdleMode(IdleMode.kCoast);
        m_frontRightMotor.burnFlash();

        m_rearLeftMotor   = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_rearLeftMotor.setInverted(Constants.Drivetrain.kRearLeftInverted);
        m_rearLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_rearLeftMotor.setIdleMode(IdleMode.kCoast);
        m_rearLeftMotor.burnFlash();

        m_rearRightMotor  = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_rearRightMotor.setInverted(Constants.Drivetrain.kRearRightInverted);
        m_rearRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_rearRightMotor.setIdleMode(IdleMode.kCoast);
        m_rearRightMotor.burnFlash();
    }

    public void driveArcade(double _straight, double _turn) {
        double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
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
        //builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
        //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
        //addChild("Controller", m_controller);
    }
}
