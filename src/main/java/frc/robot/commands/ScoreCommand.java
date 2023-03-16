package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class ScoreCommand extends ParallelDeadlineGroup {
    public ScoreCommand(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
        super(
                Commands
                        .runOnce(
                                () -> armSubsystem.setTargetPosition(Constants.Arm.kScoringPosition)
                        )
                        .andThen(new WaitCommand(2))
                        .andThen(
                                // Instant command works for gripper because the gripper updates position in its periodic
                                new InstantCommand(
                                        gripperSubsystem::openGripper,
                                        gripperSubsystem
                                )
                        )
                        .andThen(
                                new WaitCommand(0.5)
                        )
                        .andThen(
                                new InstantCommand(
                                        () -> armSubsystem.setTargetPosition(Constants.Arm.kHomePosition)
                                )
                        )
                        .andThen(
                                new WaitCommand(0.3)
                        )
                        .andThen(
                                new InstantCommand(
                                        gripperSubsystem::closeGripper,
                                        gripperSubsystem
                                )
                        )
                        .andThen(
                                new WaitCommand(1.75) // Gives the arm time to finish moving
                        )
                ,
                new RunCommand( // Runs the arm automatic to move and hold its position
                        armSubsystem::runAutomatic,
                        armSubsystem
                )
        );
    }
}