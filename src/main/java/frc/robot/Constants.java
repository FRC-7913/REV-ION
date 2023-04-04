// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
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

    public static final class AutoWait {
        public enum AutoWaitLengths {
            NONE,
            SHORT,
            LONG
        } // Don't forget to add to getTime, otherwise you'll get 0

        public static double getTime(AutoWaitLengths length) {
            switch (length) {
                case NONE:
                    return 0;
                case SHORT:
                    return 2;
                case LONG:
                    return 5;
                default:
                    return 0;
            }
        }
    }

    public static final class Drivetrain {
        public static final int kFrontLeftCanId = 1;
        public static final int kFrontRightCanId = 3;
        public static final int kRearLeftCanId = 2;
        public static final int kRearRightCanId = 4;

        public static final boolean kFrontLeftInverted = true;
        public static final boolean kFrontRightInverted = false;
        public static final boolean kRearLeftInverted = true;
        public static final boolean kRearRightInverted = false;

        public static final int kCurrentLimit = 55;

        public static final double kTurningScale = 0.5;

        /**
         * The P value used in the {@link frc.robot.commands.BalanceCommand BalanceCommand} when balancing.
         * <p>
         *     This value is multiplied by the difference between the intended pitch (0°) and the current pitch,
         *     and then fed to the motors to climb the charging station properly.
         * </p>
         * <br>
         * <p>
         *     If the robot is climbing too quickly and overshooting, consider lowering this value.
         *     If it is not making it up, a higher P value may be necessary.
         *     0.017 was used in competition to consistent (-1 instance) success.
         * </p>
         * <p>
         *     Depending on where this is being used, consider changing other variables,
         *     such as how long the robot drives for before switching to balancing.
         *     This value is not merely a speed multiplier, but rather a determiner of how well the robot achieves balancing.
         * </p>
         */
        public static final double beamBalancedDriveKP = 0.017;
        /**
         * The goal for the {@link frc.robot.commands.BalanceCommand BalanceCommand} to reach
         */
        public static final double beamBalancedGoalDegrees = 0;
        /**
         * The angle threshold below which the {@link frc.robot.commands.BalanceCommand BalanceCommand} will <strong>end</strong>
         * <p>
         *     A higher value is used to stop the robot driving as the station tips to level,
         *     instead of requiring back-and-forth balancing that can lead to failure to balance in time
         * </p>
         */
        public static final double beamBalancedAngleThresholdDegrees = 10;
        /**
         * A multiplier for speed when running the robot backwards when balancing.
         * <p>This may not be necessary, but is available in case strange weight characteristics crop up in the robot
         */
        public static final double backwardBalancingExtraPowerMultiplier = 1;

        /**
         * The angle above which the {@link frc.robot.commands.DriveOverTiltCommand DriveOverTiltCommand} considers itself not level
         */
        public static final double minOffLevelAngleDegrees = 10;
        /**
         * The threshold within which the {@link frc.robot.commands.DriveOverTiltCommand DriveOverTiltCommand} considers itself level
         */
        public static final double levelAngleThresholdDegrees = 1;
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
        public static final double kGroundConeLowPosition = 4.189;
        public static final double kGroundConeClearPosition = 3.83;
    }

    public static final class Gripper {
        public static final int kGripperCanId = 6;
        public static final double kSoftLimitReverse = -40.0;
        public static final double kSoftLimitForward = 5.0;
        public static final double kClosePosition = 0.0;
        public static final double kOpenPosition = -38.0;
        public static final double kSafePosition = -29.0;
        public static final int kCurrentLimit = 10;
        public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);
    }
}
