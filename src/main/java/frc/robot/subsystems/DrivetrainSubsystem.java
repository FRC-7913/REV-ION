// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;

public class DrivetrainSubsystem extends SubsystemBase {
    private final CANSparkMax m_frontLeftMotor;
    private final CANSparkMax m_frontRightMotor;
    private final CANSparkMax m_rearLeftMotor;
    private final CANSparkMax m_rearRightMotor;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    /** Creates a new DrivetrainSubsystem. */
    public DrivetrainSubsystem() {
        m_frontLeftMotor  = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_frontLeftMotor.setInverted(Constants.Drivetrain.kFrontLeftInverted);
        m_frontLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_frontLeftMotor.setIdleMode(IdleMode.kBrake);
        m_frontLeftMotor.burnFlash();

        m_frontRightMotor = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_frontRightMotor.setInverted(Constants.Drivetrain.kFrontRightInverted);
        m_frontRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_frontRightMotor.setIdleMode(IdleMode.kBrake);
        m_frontRightMotor.burnFlash();

        m_rearLeftMotor   = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_rearLeftMotor.setInverted(Constants.Drivetrain.kRearLeftInverted);
        m_rearLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_rearLeftMotor.setIdleMode(IdleMode.kBrake);
        m_rearLeftMotor.burnFlash();

        m_rearRightMotor  = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed);
        m_rearRightMotor.setInverted(Constants.Drivetrain.kRearRightInverted);
        m_rearRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
        m_rearRightMotor.setIdleMode(IdleMode.kBrake);
        m_rearRightMotor.burnFlash();

        leftEncoder = new Encoder(5,6, true);
        rightEncoder = new Encoder(1,2, false);

    }

    /**
     * Resets the encoders on the drivetrain
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getAverageEncoderDistanceFeetish() {

        var leftEncoderDistance = leftEncoder.getDistance();
        System.out.println("Left Encoder reported value of: " + leftEncoderDistance);

        var rightEncoderDistance = rightEncoder.getDistance();
        System.out.println("Right Encoder reported value of: " + rightEncoderDistance);

        var averageValue = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
        System.out.println("Average value of encoders is: " + averageValue);

        /* This is a complete voodoo magic formula
        * Determined to work when asking robot to drive at 0.3 speed
        * Tested on distances around 5 ft, though accuracy shouldn't decay too greatly past that
        * The slip of the connection to the motor can be handled alright with the average
        *  but the numbers from the encoders start to diverge after a while
        * This might require using only the left encoder, since that one seems to be consistent
        * The 0.66 was to account for a consistent 8-inch offset
        * This may need to be scaled based on speed,
        *  which probably means adding a variable
        */
        var distanceFeet = averageValue / 1200;
        System.out.println("Reporting distance of: " + distanceFeet);

        System.out.println();

        return distanceFeet;
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
