package frc.robot.commands;

// Based on https://github.com/GOFIRST-Robotics/Ri3D-2023/blob/main/src/main/java/frc/robot/commands/BalanceOnBeamCommand.java

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class BalanceCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrainSubsystem;

    private double error;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance */
    public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(this.drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
        // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
        double currentAngle = drivetrainSubsystem.getPitch();

        error = Constants.Drivetrain.beamBalancedGoalDegrees - currentAngle;
        double drivePower = -Math.min(Constants.Drivetrain.beamBalancedDriveKP * error, 1);

        // Source robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower < 0) {
            drivePower *= Constants.Drivetrain.backwardBalancingExtraPowerMultiplier;
        }

        // Limit the max power
        if (Math.abs(drivePower) > 0.4) {
            drivePower = Math.copySign(0.4, drivePower);
        }

        drivetrainSubsystem.tankDrive(drivePower, drivePower);

        // Debugging Print Statements
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Error " + error);
        System.out.println("Drive Power: " + drivePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.tankDrive(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(error) < Constants.Drivetrain.beamBalancedAngleThresholdDegrees; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    }
}