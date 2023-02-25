// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.util.sendable.SendableBuilder;

public class DrivetrainSubsystem extends SubsystemBase {
    private final MotorControllerGroup m_leftMotors =
            new MotorControllerGroup(
                new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed),
                new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushed)
            );
    private final MotorControllerGroup m_rightMotors =
            new MotorControllerGroup(
                    new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed),
                    new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushed)
            );

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The left-side drive encoder
    private final Encoder m_leftEncoder = new Encoder(
           Constants.Drivetrain.kLeftEncoderPorts[0],
           Constants.Drivetrain.kLeftEncoderPorts[1],
           Constants.Drivetrain.kLeftEncoderReversed
    );

    // The right-side drive encoder
    private final Encoder m_rightEncoder = new Encoder(
        Constants.Drivetrain.kRightEncoderPorts[0],
        Constants.Drivetrain.kRightEncoderPorts[1],
        Constants.Drivetrain.kRightEncoderReversed
    );

    private final AHRS m_navX = new AHRS(SPI.Port.kMXP);

    private final Gyro m_gyro = m_navX;

    private final DifferentialDriveOdometry m_odometry;

    /** Creates a new DrivetrainSubsystem. */
    public DrivetrainSubsystem() {
        m_leftMotors.setInverted(Constants.Drivetrain.kLeftInverted);
        m_rightMotors.setInverted(Constants.Drivetrain.kRightInverted);

        m_leftEncoder.setDistancePerPulse(Constants.Drivetrain.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(Constants.Drivetrain.kEncoderDistancePerPulse);

        resetEncoders();
        m_odometry =
                new DifferentialDriveOdometry(
                        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance()
                );
    }

    public void driveArcade(double _straight, double _turn) {
        double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
        double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);

        m_leftMotors.set(left);
        m_rightMotors.set(right);
    }

    public void tankDrive(double _left, double _right) {
        m_leftMotors.set(_left);
        m_rightMotors.set(_right);
    }

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if(isFirstPath){
                        this.resetOdometry(traj.getInitialPose());
                    }
                }),
                new PPRamseteCommand(
                        traj,
                        this::getPose, // Pose supplier
                        new RamseteController(),
                        new SimpleMotorFeedforward(
                                Constants.Drivetrain.ksVolts,
                                Constants.Drivetrain.kvVoltSecondsPerMeter,
                                Constants.Drivetrain.kaVoltSecondsSquaredPerMeter
                        ),
                        Constants.Drivetrain.kDriveKinematics, // DifferentialDriveKinematics
                        this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                        new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                        this::tankDriveVolts, // Voltage biconsumer
                        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                        this // Requires this drive subsystem
                )
        );
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(
                m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_drive.feed();
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

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(
                m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance()
        );
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
