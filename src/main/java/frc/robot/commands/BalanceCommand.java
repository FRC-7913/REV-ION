package frc.robot.commands;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends SequentialCommandGroup {
    public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem, AHRS ahrs) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                Commands.waitUntil(() -> (ahrs.getPitch() > 3 || ahrs.getPitch() < -3))
                        .deadlineWith(new RunCommand(
                                () -> drivetrainSubsystem.driveArcade(0.5,0),
                                drivetrainSubsystem
                        )),
                Commands.either(
                        Commands.run(() -> {
                            if (ahrs.getPitch() < -3)
                                drivetrainSubsystem.driveArcade(-0.2, 0);
                            else if (ahrs.getPitch() > 3)
                                drivetrainSubsystem.driveArcade(0.2, 0);
                        }, drivetrainSubsystem),
                        Commands.run(() -> drivetrainSubsystem.driveArcade(0,0), drivetrainSubsystem),
                        () -> (ahrs.getPitch() > 3 || ahrs.getPitch() < -3)
                )
        );
    }
}