package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public VisionSubsystem() {
        limelight.getEntry("ledMode").setNumber(0);; // LED on
        setCamMode(0); // Vision processing mode
    }

    public Pose2d getEstimatedPose() {
        double[] botPose = getBotPoseArray();
        double tx = botPose[0];  // X translation
        double ty = botPose[1];  // Y translation
        double ta = botPose[5];  // Rotation around Z-axis

        // alliance mirroring idk ???

        return new Pose2d(
            new Translation2d(tx, ty),
            new Rotation2d(Math.toRadians(ta))
        );
    }

    private double[] getBotPoseArray() {
        return limelight.getEntry("botpose").getDoubleArray(new double[6]);
    }

    public boolean hasValidTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public void toggleCameraMode() {
        int currentMode = (int) limelight.getEntry("camMode").getDouble(0);
        setCamMode(currentMode == 0 ? 1 : 0);   // 0 is vision 1 is driver
    }

    public void setCamMode(int mode) {
        limelight.getEntry("camMode").setNumber(mode);
    }

    public void setPipeLine(int pipeline){
        limelight.getEntry("pipeline").setNumber(pipeline);
    }
}