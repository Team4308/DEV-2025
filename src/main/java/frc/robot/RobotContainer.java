// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.score;
import frc.robot.commands.Sequential.CoralIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotContainer {
  public static boolean groundIntake = false; 
  private final EndEffectorSubsystem m_intakeSubsystem = new EndEffectorSubsystem();
  private final DriveSystem m_driveSystem = new DriveSystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final VisionSubsystem m_Vision = new VisionSubsystem(m_driveSystem.leftEncoder, m_driveSystem.rightEncoder);
  @SuppressWarnings("static-access")
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_driveSystem.leaderLeft, m_driveSystem.leaderRight);

  private final XBoxWrapper driver = new XBoxWrapper(Ports.Joysticks.DRIVER);
  private final XBoxWrapper operator = new XBoxWrapper(Ports.Joysticks.OPERATOR);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Register subsystems
    CommandScheduler.getInstance().registerSubsystem(m_armSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_driveSystem);
    CommandScheduler.getInstance().registerSubsystem(m_Vision);
    configureNamedCommands();
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    DriverBinds();
    OperatorBinds();
  }

  private void DriverBinds() {
    m_driveSystem.setDefaultCommand(
      new RunCommand(
        () -> {
          double xSpeed = -driver.getLeftY();
          double zRotation = driver.getLeftX();
        
          m_robotDrive.arcadeDrive(xSpeed, zRotation, true);


        },
        m_driveSystem
      )
    );
  }
  public void configureNamedCommands() {
    NamedCommands.registerCommand("intake", new CoralIntakeCommand(m_intakeSubsystem, m_armSubsystem, groundIntake));
    NamedCommands.registerCommand("score", new score(m_intakeSubsystem, m_armSubsystem));
  }

  private void OperatorBinds() {

   // If we dont have a coral try to intake it. If we do have a coral score it.

   // This could also just all be in driver. Change ltr
   operator.B.onTrue(new InstantCommand(() -> { groundIntake = !groundIntake;  }));

   if (!m_intakeSubsystem.intakeSensor.get()) { 
    operator.A.onTrue(new CoralIntakeCommand(m_intakeSubsystem, m_armSubsystem, groundIntake));
   } else {
    operator.A.onTrue(new score(m_intakeSubsystem, m_armSubsystem));
   }

  }

  public static boolean groundIntake() {
    return groundIntake;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}