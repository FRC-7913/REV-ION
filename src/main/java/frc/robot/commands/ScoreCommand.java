package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                // Run command needed to continuously update arm (command repeats until finished)
                new RunCommand(
                        () -> {armSubsystem.setTargetPosition(Constants.Arm.kScoringPosition, gripperSubsystem);},
                        armSubsystem
                ),
                // Instant command works for gripper because the gripper updates position in its periodic
                new InstantCommand(
                        gripperSubsystem::openGripper,
                        gripperSubsystem
                ),
                new ParallelCommandGroup(
                        new RunCommand(
                                () -> {armSubsystem.setTargetPosition(Constants.Arm.kHomePosition, gripperSubsystem);},
                                armSubsystem
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(0.2),
                                new InstantCommand(
                                        gripperSubsystem::closeGripper,
                                        gripperSubsystem
                                )
                        )
                )
        );
    }
}