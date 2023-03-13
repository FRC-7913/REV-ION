package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistanceCommand extends FunctionalCommand {
    /**
     * Creates a new command that will drive the robot a certain distance
     * @param distanceFeet The distance to drive
     * @param speed The speed at which to drive. 0.3 has been proven fairly accurate
     *              DO NOT MAKE NEGATIVE TO DRIVE BACKWARDS
     * @param backwards Whether the robot drives backwards
     * @param drivetrainSubsystem A reference to the DrivetrainSubsystem
     */
    public DriveDistanceCommand(double distanceFeet, double speed, boolean backwards, DrivetrainSubsystem drivetrainSubsystem) {
        super(
                drivetrainSubsystem::resetEncoders,
                () -> drivetrainSubsystem.driveArcade(
                        backwards ?
                                -Math.abs(speed) :
                                Math.abs(speed),
                        0),
                interrupted -> drivetrainSubsystem.driveArcade(0,0),
                () -> backwards ?
                        drivetrainSubsystem.getAverageEncoderDistanceFeet() <= -distanceFeet : // Going backwards
                        drivetrainSubsystem.getAverageEncoderDistanceFeet() >= distanceFeet, // Going forwards
                drivetrainSubsystem
        );
    }
}
