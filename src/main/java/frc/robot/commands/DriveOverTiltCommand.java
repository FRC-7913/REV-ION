package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveOverTiltCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private double speed = 0.4;
    private double time = 0.0;
    private final double maxFlatTime;
    private boolean hasTilted = false;

    public DriveOverTiltCommand(DrivetrainSubsystem drivetrainSubsystem, int direction, double flatTime) {

        maxFlatTime = flatTime;
        speed = Math.copySign(speed, direction);

        this.drivetrainSubsystem = drivetrainSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.zeroGyro();
    }

    @Override
    public void execute() {
        drivetrainSubsystem.driveArcade(speed, 0);
    }

    @Override
    public boolean isFinished() {

        double pitch = drivetrainSubsystem.getPitch();

        if (!hasTilted)
            hasTilted = pitch > Constants.Drivetrain.minOffLevelAngleDegrees ||
                    pitch < -Constants.Drivetrain.minOffLevelAngleDegrees;

        else
            time += (pitch < Constants.Drivetrain.levelAngleThresholdDegrees ||
                    pitch > -Constants.Drivetrain.levelAngleThresholdDegrees) ?
                    0.02 : 0; // Adds 0.02 seconds (loop time for functions) whenever the angle is outside the threshold

        return (time >= maxFlatTime);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
