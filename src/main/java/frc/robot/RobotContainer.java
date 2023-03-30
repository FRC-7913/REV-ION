// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    public final SendableChooser<Constants.AutoWait.AutoWaitLengths> autoWaitLengthsChooser = new SendableChooser<>();

    // The robot's subsystems and commands are defined here...
    //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private final GripperSubsystem m_gripper = new GripperSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

    private final XboxController m_driveController = new XboxController(Constants.OIConstants.kDriverController);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        m_drivetrain.zeroGyro();

        for (Constants.AutoWait.AutoWaitLengths length: Constants.AutoWait.AutoWaitLengths.values()
             ) {
            String name = length.name(); // SHORT
            name = name.substring(0,1).toUpperCase() + name.substring(1).toLowerCase(); // Short
            name += " (" + Constants.AutoWait.getTime(length) + ")"; // Short (2)

            if (length == Constants.AutoWait.AutoWaitLengths.NONE) {
                autoWaitLengthsChooser.setDefaultOption(name, length);
                continue;
            }

            autoWaitLengthsChooser.addOption(name, length);
        }

        SmartDashboard.putData("Autonomous Delay (seconds)", autoWaitLengthsChooser);

        // For each autonomous command
        // autonomousChooser.addOption(NAME, COMMAND);
        autonomousChooser.addOption(
                "Short Side Auto",
                new ScoreCommand(m_arm, m_gripper)
                        .andThen(new WaitCommand(0))
                        .andThen(new DriveDistanceCommand(7.5, 0.3, true, m_drivetrain))
        );
        autonomousChooser.addOption(
                "Long Side Auto",
                new ScoreCommand(m_arm, m_gripper)
                        .andThen(new WaitCommand(0))
                        .andThen(new RunCommand(() -> m_drivetrain.driveArcade(-0.6, 0)))
                        .withTimeout(1.5)
        );
        autonomousChooser.addOption(
                "Over Charge and Dock",
                new ScoreCommand(m_arm, m_gripper)
                        .alongWith(
                                new WaitCommand(4)
                                .andThen(new DriveOverTiltCommand(m_drivetrain, -1, 1.5))
                        )
                        .andThen(new WaitCommand(1))
                        .andThen(new DriveUntilTiltCommand(m_drivetrain, 1, 1.5))
                        .andThen(new BalanceCommand(m_drivetrain))
        );
        autonomousChooser.addOption(
                "Dock",
                new ScoreCommand(m_arm, m_gripper)
                        .andThen(new DriveUntilTiltCommand(m_drivetrain, -1, 1))
                        .andThen(new BalanceCommand(m_drivetrain))
        );
        autonomousChooser.addOption(
                "Just Score",
                new ScoreCommand(m_arm, m_gripper)
        );

        autonomousChooser.setDefaultOption("Do Nothing", new PrintCommand("Did nothing as an autonomous"));

        SmartDashboard.putData(autonomousChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //set up the drivetrain command that runs all the time
        m_drivetrain.setDefaultCommand(new RunCommand(
                () ->
                        m_drivetrain.driveArcade(
                                MathUtil.applyDeadband(-m_driveController.getLeftY(), Constants.OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(m_driveController.getRightX(), Constants.OIConstants.kDriveDeadband) * Constants.Drivetrain.kTurningScale)
                , m_drivetrain)
        );

        //set up gripper open/close
        new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(m_gripper::openGripper))
                .onFalse(new InstantCommand(m_gripper::closeGripper));

        //set up arm preset positions
        new JoystickButton(m_driveController, XboxController.Button.kA.value)
                .onTrue(new InstantCommand(() -> {
                    m_gripper.closeGripper();
                    m_arm.setTargetPosition(Constants.Arm.kHomePosition);
                }));
        new JoystickButton(m_driveController, XboxController.Button.kX.value)
                .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
        new JoystickButton(m_driveController, XboxController.Button.kY.value)
                .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
        new JoystickButton(m_driveController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kFeederPosition)));

        new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value)
                .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kGroundConeLowPosition)))
                .toggleOnFalse(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kGroundConeClearPosition)));

        new JoystickButton(m_driveController, XboxController.Button.kStart.value)
                .whileTrue(new BalanceCommand(m_drivetrain));

        //set up arm manual and auto functions
        m_arm.setDefaultCommand(new RunCommand(
                m_arm::runAutomatic
                , m_arm)
        );
        new Trigger(() ->
                Math.abs(m_driveController.getRightTriggerAxis() - m_driveController.getLeftTriggerAxis()) > Constants.OIConstants.kArmManualDeadband
        ).whileTrue(new RunCommand(
                () ->
                        m_arm.runManual((m_driveController.getRightTriggerAxis() - m_driveController.getLeftTriggerAxis()) * Constants.OIConstants.kArmManualScale)
                , m_arm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new WaitCommand(
                Constants.AutoWait.getTime(autoWaitLengthsChooser.getSelected())
        ).andThen(
                autonomousChooser.getSelected()
        );
    }
}