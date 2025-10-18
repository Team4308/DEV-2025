package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class DriveToCoralCommand extends Command {
    private final DriveSystem drive;
    private final VisionSubsystem vision;
    private final EndEffectorSubsystem intake;
    private final Timer timer = new Timer();

    // Keep last area to decide proximity
    private double lastArea = 0.0;

    public DriveToCoralCommand(DriveSystem drive, VisionSubsystem vision, EndEffectorSubsystem intake) {
        this.drive = drive;
        this.vision = vision;
        this.intake = intake;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        lastArea = 0.0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        LimelightHelpers.RawDetection det = vision.getBestCoralDetection();

        if (det != null) {
            lastArea = det.ta;
            double steerErr = det.txnc; 
            double turn = Constants.Vision.seekSteerKp * steerErr;

            double fwd = Math.abs(steerErr) < Constants.Vision.seekAlignDeadband
                ? Constants.Vision.seekMaxFwd
                : Math.max(Constants.Vision.seekMinFwd, Constants.Vision.seekMaxFwd * (1.0 - Math.abs(steerErr)));

            double left = fwd + turn;
            double right = fwd - turn;
            drive.setPowerPercent(left, right);
        } else {
            drive.setPowerPercent(Constants.Vision.seekLostRotate, -Constants.Vision.seekLostRotate);
        }
    }

    @Override
    public boolean isFinished() {
        boolean intakeHasPiece = (intake != null) && (intake.intakeSensor != null) && !intake.intakeSensor.get();
        boolean closeEnough = lastArea >= Constants.Vision.seekStopArea;
        boolean timedOut = timer.hasElapsed(Constants.Vision.seekTimeoutSec);
        return intakeHasPiece || closeEnough || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        drive.StopControllers();
    }
}
