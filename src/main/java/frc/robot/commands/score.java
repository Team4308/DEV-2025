package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class score  extends SequentialCommandGroup {
        public score(EndEffectorSubsystem endEffector, ArmSubsystem arm) {
        addCommands(
            new InstantCommand(() -> arm.moveToScoringPosition(), arm),
            new WaitUntilCommand(() -> Math.abs(arm.getCurrentAngle() - arm.getTargetAngle()) < 1.0),
            new InstantCommand(() -> endEffector.outtake(), endEffector),
            new WaitUntilCommand(() -> !endEffector.intakeSensor.get()).withTimeout(3),
            new InstantCommand(endEffector::stop),
            new InstantCommand(() -> arm.moveToRestingPosition(), arm)
        );
    }
}