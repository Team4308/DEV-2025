package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;


public class CoralIntakeCommand extends SequentialCommandGroup {
    public CoralIntakeCommand(EndEffectorSubsystem endEffector, ArmSubsystem arm, boolean groundIntake) {
        if (groundIntake) {
            addCommands(
                new InstantCommand(() -> arm.moveToGroundIntakePosition(), arm)
            );
        } else {
            addCommands(
                new InstantCommand(() -> arm.moveToFeederPosition(), arm)
            );
        }
        addCommands(
            new intake(endEffector)
        );

    }
    
}
