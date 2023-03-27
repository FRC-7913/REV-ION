package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveUntilTiltCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private double speed = 0.4;

    public DriveUntilTiltCommand(DrivetrainSubsystem drivetrainSubsystem, int direction) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrainSubsystem);
        speed = Math.copySign(speed, direction);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.zeroGyro();
    }

    @Override
    public void execute() {
        drivetrainSubsystem.driveArcade(speed,0);
    }

    private double time = 0.0;

    @Override
    public boolean isFinished() {

        if (drivetrainSubsystem.getPitch() > 3 || drivetrainSubsystem.getPitch() < -3) {
            time += 0.02;
            if (time > 1) {
                return drivetrainSubsystem.getPitch() > 3 || drivetrainSubsystem.getPitch() < -3;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
