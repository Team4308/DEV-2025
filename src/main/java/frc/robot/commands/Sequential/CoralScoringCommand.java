package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.commands.score;

public class CoralScoringCommand extends SequentialCommandGroup {
    public CoralScoringCommand(EndEffectorSubsystem intake, ArmSubsystem arm) {
        addCommands(
            new InstantCommand(arm::moveToScoringPosition, arm),
            new WaitCommand(0.25),
            new score(intake, arm)
        );
    }
}
