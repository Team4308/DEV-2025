package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class intake extends SequentialCommandGroup {
    public intake(EndEffectorSubsystem endEffector) {
        addCommands(
            new InstantCommand(endEffector::intake),
            new WaitCommand(1).andThen(new InstantCommand(endEffector::stop))
        );
    }
}
