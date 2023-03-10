// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  private final GripperSubsystem m_gripper = new GripperSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final FunctionalCommand kDisabledAuto = null;
  private final FunctionalCommand kSideAuto = new FunctionalCommand(
      // Reset encoders on command start
      m_drivetrain::resetEncoders,
      // Start driving forward at the start of the command
      () -> m_drivetrain.driveArcade(-0.2, 0),
      // Stop driving at the end of the command
      interrupted -> m_drivetrain.driveArcade(0, 0),
      // End the command when the robot's driven distance exceeds the desired value
      () -> m_drivetrain.getLeftEncoderDistance() >= 9250,
      // Require the drive subsystem
      m_drivetrain);

      private final SequentialCommandGroup kMiddleAuto = new SequentialCommandGroup(
        new InstantCommand(m_drivetrain::resetEncoders, m_drivetrain),
        new WaitCommand(2),
        new InstantCommand(() -> m_drivetrain.driveArcade(-0.8,0),m_drivetrain),
        new WaitUntilCommand(() -> (m_drivetrain.getLeftEncoderDistance() >= 9500)),
        new InstantCommand(() -> m_drivetrain.driveArcade(0,0), m_drivetrain)
      );
        /* 
      private final FunctionalCommand kMiddleAuto = new FunctionalCommand(
      // Reset encoders on command start
      m_drivetrain::resetEncoders,
      Timer.delay(2.0),
      // Start driving forward at the start of the command
      () -> m_drivetrain.driveArcade(-0.8, 0),
      // Stop driving at the end of the command
      interrupted -> m_drivetrain.driveArcade(0, 0),
      // End the command when the robot's driven distance exceeds the desired value
      () -> m_drivetrain.getLeftEncoderDistance() >= 9500,
      // Require the drive subsystem
      m_drivetrain);
      */

  private XboxController m_driveController = new XboxController(Constants.OIConstants.kDriverController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("DISABLED Auto", kDisabledAuto);
    m_chooser.addOption("Side Auto", kSideAuto);
    m_chooser.addOption("Middle Auto", kMiddleAuto);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // set up the drivetrain command that runs all the time
    m_drivetrain.setDefaultCommand(new RunCommand(
        () -> m_drivetrain.driveArcade(
            MathUtil.applyDeadband(-m_driveController.getLeftY(), Constants.OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(m_driveController.getRightX() * Constants.Drivetrain.kTurningScale,
                Constants.OIConstants.kDriveDeadband)),
        m_drivetrain));

    // set up gripper open/close
    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_gripper.openGripper()))
        .onFalse(new InstantCommand(() -> m_gripper.closeGripper()));

    // set up arm preset positions
    new JoystickButton(m_driveController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition, m_gripper)));
    new JoystickButton(m_driveController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition, m_gripper)));
    new JoystickButton(m_driveController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition, m_gripper)));
    new JoystickButton(m_driveController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kFeederPosition, m_gripper)));

    // set up arm manual and auto functions
    m_arm.setDefaultCommand(new RunCommand(
        () -> m_arm.runAutomatic(), m_arm));
    new Trigger(() -> Math.abs(m_driveController.getRightTriggerAxis()
        - m_driveController.getLeftTriggerAxis()) > Constants.OIConstants.kArmManualDeadband).whileTrue(new RunCommand(
            () -> m_arm.runManual((m_driveController.getRightTriggerAxis() - m_driveController.getLeftTriggerAxis())
                * Constants.OIConstants.kArmManualScale),
            m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
