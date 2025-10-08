package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class intake extends SequentialCommandGroup {
    public intake(EndEffectorSubsystem endEffector) {
        addCommands(
            new InstantCommand(endEffector::intake),
            new WaitUntilCommand(() -> endEffector.intakeSensor.get()
            ).withTimeout(5),
            new InstantCommand(endEffector::stop)
        );
    }
}
