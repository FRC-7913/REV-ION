package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveOverTiltCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private double speed = 0.4;
    /**
     * The time for which the robot has been at a non-zero pitch.
     * <p>
     * As with the {@link DriveUntilTiltCommand}, time elapsed (based on System.nanoTime()) is added to this on every iteration of the function
     */
    private long time = 0;
    /**
     * A snapshot of the last System.nanoTime() when it was measured
     */
    private long deltaTime = 0;
    private final long maxFlatTime;
    private boolean hasTilted = false;

    /**
     * Constructs a command that will drive the robot over a tilted surface (the charging station)
     * @param drivetrainSubsystem The drivetrain for the robot, for driving and gyro access
     * @param direction The direction in which to drive. A negative number will drive backwards, a positive one forwards
     * @param flatTime The time (based on loop iterations running at 0.02 seconds) that the robot needs to be flat before stopping
     */
    public DriveOverTiltCommand(DrivetrainSubsystem drivetrainSubsystem, int direction, double flatTime) {

        maxFlatTime = (long) (flatTime * 1000000000);
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

        if (deltaTime == 0) deltaTime = System.nanoTime();

        if (!hasTilted)
            hasTilted = pitch > Constants.Drivetrain.minOffLevelAngleDegrees ||
                    pitch < -Constants.Drivetrain.minOffLevelAngleDegrees;

        else
            time += (pitch < Constants.Drivetrain.levelAngleThresholdDegrees ||
                    pitch > -Constants.Drivetrain.levelAngleThresholdDegrees) ?
                    System.nanoTime() - deltaTime : 0; // Adds 0.02 seconds (loop time for functions) whenever the angle is outside the threshold

        deltaTime = System.nanoTime();

        return (time >= maxFlatTime);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
