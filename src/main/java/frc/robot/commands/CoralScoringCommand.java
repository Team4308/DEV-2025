package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ScoreL1Command extends SequentialCommandGroup {

    public ScoreL1Command(ArmSubsystem armSubsystem, EndEffectorSubsystem endEffector) {
        addCommands(
            new InstantCommand(armSubsystem::moveToScoringPosition),
            new WaitCommand(1),
            new InstantCommand(endEffector::outtake),
            new WaitCommand(1),
            new InstantCommand(endEffector::stop),
            new WaitCommand(1),
            new InstantCommand(armSubsystem::moveToRestingPosition)
        );
    }
}
