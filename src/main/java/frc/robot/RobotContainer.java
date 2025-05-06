// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.DriveSystem;
import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotContainer {
  private final EndEffectorSubsystem m_intakeSubsystem;
  private final DriveSystem m_driveSystem;

  private final DifferentialDrive m_robotDrive;

  private final XBoxWrapper driver = new XBoxWrapper(Ports.Joysticks.DRIVER);
  private final XBoxWrapper operator = new XBoxWrapper(Ports.Joysticks.OPERATOR);
  //private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Register the subsystems
    m_intakeSubsystem = new EndEffectorSubsystem();
    m_driveSystem = new DriveSystem();
    
    m_robotDrive = new DifferentialDrive(m_driveSystem.leaderLeft, m_driveSystem.leaderRight);



    CommandScheduler.getInstance().registerSubsystem(m_intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_driveSystem);

    configureBindings();
    
    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    DriverBinds();
    OperatorBinds();
  }

  private void DriverBinds() {
    m_driveSystem.setDefaultCommand(
      new RunCommand(
        () -> {
          double throttle = -driver.getLeftY();
          double turn = driver.getLeftX();   
         m_robotDrive.arcadeDrive(throttle, turn, false); 
        },
        m_driveSystem
      )
    );
  }
  

  private void OperatorBinds() {
    operator.A.onTrue(new InstantCommand(() -> m_intakeSubsystem.intake()));
    operator.A.onFalse(new InstantCommand(() -> m_intakeSubsystem.stop()));
    operator.B.onTrue(new InstantCommand(() -> m_intakeSubsystem.outtake()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
