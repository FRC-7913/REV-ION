package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveUntilTiltCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    /**
     * The speed (direction is copied in the constructor) that the robot will drive at while executing the command
     */
    private double speed = 0.4;
    /**
     * The time for which the robot has been at a non-zero pitch.
     * <p>
     * As with the {@link DriveOverTiltCommand}, time elapsed (based on System.nanoTime()) is added to this on every iteration of the function
     */
    private long time = 0;
    /**
     * A snapshot of the last System.nanoTime() when it was measured
     */
    private long deltaTime = 0;

    private final long drivePastNanoseconds;

    /**
     * Drives the robot until it reaches an incline
     * @param drivetrainSubsystem The drivetrain for the robot, for driving and gyro access
     * @param direction The direction in which to drive. A negative number will drive backwards, a positive one forwards
     * @param _drivePastSeconds How many seconds (calculated based on command scheduler loops of 0.02 seconds) past when an incline is detected that the robot should continue driving
     */
    public DriveUntilTiltCommand(DrivetrainSubsystem drivetrainSubsystem, int direction, double _drivePastSeconds) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrainSubsystem);
        speed = Math.copySign(speed, direction);
        drivePastNanoseconds = (long) (_drivePastSeconds * 1000000000);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrainSubsystem.driveArcade(speed,0);
    }


    @Override
    public boolean isFinished() {

        if (drivetrainSubsystem.getPitch() > 3 || drivetrainSubsystem.getPitch() < -3) {
            if (deltaTime == 0) deltaTime = System.nanoTime();
            time += System.nanoTime() - deltaTime;
            deltaTime = System.nanoTime();
            if (time > drivePastNanoseconds) {
                return drivetrainSubsystem.getPitch() > 3 || drivetrainSubsystem.getPitch() < -3;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
