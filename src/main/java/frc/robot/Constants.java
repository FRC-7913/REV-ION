// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.PIDGains;
import java.lang.Math;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        public static final int kDriverController = 0;
        public static final double kDriveDeadband = 0.05;
        public static final double kArmManualDeadband = 0.05;
        public static final double kArmManualScale = 0.5;
    }

    public static final class Drivetrain {
        public static final int kFrontLeftCanId = 1;
        public static final int kFrontRightCanId = 3;
        public static final int kRearLeftCanId = 2;
        public static final int kRearRightCanId = 4;

        public static final int[] kLeftEncoderPorts = new int[]{ 1, 2 };
        public static final boolean kLeftEncoderReversed = false;
        public static final int[] kRightEncoderPorts = new int[]{ 5, 6 };
        public static final boolean kRightEncoderReversed = true;

        public static final Double kEncoderDistancePerPulse = 1.0 / 256.0;

        public static final boolean kLeftInverted = true;
        public static final boolean kRightInverted = false;

        public static final int kCurrentLimit = 55;

        public static final double kTurningScale = 0.5;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    }

    public static final class Arm {
        public static final int kArmCanId = 5;
        public static final boolean kArmInverted = false;
        public static final int kCurrentLimit = 40;

        public static final double kSoftLimitReverse = 0.0;
        public static final double kSoftLimitForward = 4.6;

        public static final double kArmGearRatio = 1.0 / (48.0 * 4.0);
        public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
        public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
        public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
        public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
        public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
        public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
        public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

        public static final double kHomePosition = 0.0;
        public static final double kScoringPosition = 3.05;
        public static final double kIntakePosition = 4.52;
        public static final double kFeederPosition = 2.95;
    }

    public static final class Gripper {
        public static final int kGripperCanId = 6;
        public static final double kSoftLimitReverse = -34.0;
        public static final double kSoftLimitForward = 5.0;
        public static final double kClosePosition = 0.0;
        public static final double kOpenPosition = -34.0;
        public static final double kSafePosition = -29.0;
        public static final int kCurrentLimit = 10;
        public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);
    }
}
