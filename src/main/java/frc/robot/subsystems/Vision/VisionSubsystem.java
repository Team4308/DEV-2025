package frc.robot.subsystems.Vision;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers.RawDetection;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.Timer;
import static java.lang.Double.MAX_VALUE;
import java.util.Arrays;
import java.util.Comparator;

public class VisionSubsystem extends SubsystemBase {
    // Hardware & Estimator
    public final AHRS gyro;

    // NetworkTables
    private final NetworkTable limelightTable;
    
    // State
    private CameraMode currentCamMode;
    private Pipeline currentPipeline;
    private Pose2d lastEstimatedPose;
    private Pose2d lastVisionPose; // latest raw vision measurement used
    private int lastTargetCount;

    private double lastSimVisionTimestamp = -1.0;

    private final DriveSystem drive;

    /**enum for limelight network table camera mode configuration. CameraMode.VISION or CameraMode.DRIVER*/
    public enum CameraMode { VISION(0), DRIVER(1); public final int value; CameraMode(int v) { value = v; }}

    /**enum for limelight network table pipeline configuration. Pipeline.APRIL_TAGS or Pipeline.OBJECT_DETECTION*/
    public enum Pipeline { APRIL_TAGS(0), OBJECT_DETECTION(1); public final int value; Pipeline(int v) { value = v; }}



    /** Constructor: configure sensors, estimator, and Limelight */
    public VisionSubsystem(DriveSystem drive) {
        // Initialize kinematics and sensors
        gyro = DriveSystem.imu;
        this.drive = drive;

        // general setup
        currentCamMode = CameraMode.VISION;
        currentPipeline = Pipeline.APRIL_TAGS;
        lastEstimatedPose = new Pose2d();
        lastTargetCount = 0;
        limelightTable = NetworkTableInstance.getDefault().getTable(Constants.Vision.LIMELIGHT_TABLE_NAME);

        // Apply initial configuration
        switchCameraMode(CameraMode.VISION);
        switchPipeline(Pipeline.APRIL_TAGS);
        LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.LIMELIGHT_TABLE_NAME);
        LimelightHelpers.setCameraPose_RobotSpace(
            Constants.Vision.LIMELIGHT_TABLE_NAME,
            Constants.Vision.camOffsetFront,
            Constants.Vision.camOffsetSide,
            Constants.Vision.camOffsetUp,
            Constants.Vision.camOffsetRoll,
            Constants.Vision.camOffsetPitch,
            Constants.Vision.camOffsetYaw
        );
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /** 
     * Update the pose estimator with odometry and vision data if valid. 
     * Computes robot global position with blue side left as origin.
     * Should be called periodically.
    */
    public void updatePose() {
        // --- VISION UPDATE ---
        switchPipeline(Pipeline.APRIL_TAGS);
        switchCameraMode(CameraMode.VISION);

        boolean hasGoodVision = false;

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            SmartDashboard.putNumberArray("LL Raw BotPose", botpose);
            double tv = limelightTable.getEntry("tv").getDouble(0.0);
            int tc = (int) limelightTable.getEntry("tc").getDouble(0.0);
            lastTargetCount = tc;

            if (tv >= 1.0 && botpose.length >= 7 && tc >= Constants.Vision.minTagsForVision) {
                Pose2d simPose = new Pose2d(
                    botpose[0],
                    botpose[1],
                    Rotation2d.fromDegrees(botpose[5])
                );
                double latencySec = botpose[6] / 1000.0;
                double ts = Timer.getFPGATimestamp() - latencySec;

                if (ts > lastSimVisionTimestamp && isPoseCloseToOdom(simPose)) {
                    drive.addVisionMeasurement(simPose, ts);
                    lastVisionPose = simPose;
                    lastSimVisionTimestamp = ts;
                    hasGoodVision = true;
                } else {
                    lastVisionPose = null;
                }
            } else {
                lastVisionPose = null;
            }
        } else {
            double gyroYawDeg = gyro.getRotation2d().getDegrees();
            LimelightHelpers.SetRobotOrientation(
                Constants.Vision.LIMELIGHT_TABLE_NAME, gyroYawDeg, 0, 0, 0, 0, 0
            );

            LimelightHelpers.PoseEstimate visionResult =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.LIMELIGHT_TABLE_NAME);

            double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            SmartDashboard.putNumberArray("LL Raw BotPose", botpose);

            if (visionResult != null) {
                SmartDashboard.putNumber("LL Vision TagCount", visionResult.tagCount);

                boolean validVision = hasValidAprilTarget()
                    && Math.abs(gyro.getRate()) < Constants.Vision.maxGyroRate
                    && visionResult.tagCount > 0
                    && !(visionResult.pose.getX() == 0 && visionResult.pose.getY() == 0);

                if (validVision && visionResult.tagCount >= Constants.Vision.minTagsForVision
                    && isPoseCloseToOdom(visionResult.pose)) {
                    drive.addVisionMeasurement(visionResult.pose, visionResult.timestampSeconds);
                    lastVisionPose = visionResult.pose;
                    hasGoodVision = true;
                } else {
                    lastVisionPose = null;
                }
            } else {
                lastVisionPose = null;
            }
        }

        lastEstimatedPose = drive.getPose();

        if (hasGoodVision && lastVisionPose != null) {
            double[] visionPose2d = {
                lastVisionPose.getX(),
                lastVisionPose.getY(),
                lastVisionPose.getRotation().getDegrees()
            };
            NetworkTableInstance.getDefault()
                .getTable("/AdvantageScope/Vision")
                .getEntry("VisionPose")
                .setDoubleArray(visionPose2d);
        } else {
            NetworkTableInstance.getDefault()
                .getTable("/AdvantageScope/Vision")
                .getEntry("VisionPose")
                .setDoubleArray(new double[0]);
        }

        double alpha = hasGoodVision ? 0.75 : 0.25;
        Pose2d blended = new Pose2d(
            lastEstimatedPose.getTranslation().interpolate(
                (lastVisionPose != null ? lastVisionPose.getTranslation() : lastEstimatedPose.getTranslation()),
                alpha
            ),
            lastEstimatedPose.getRotation().interpolate(
                (lastVisionPose != null ? lastVisionPose.getRotation() : lastEstimatedPose.getRotation()),
                alpha
            )
        );
        double[] blended2d = {
            blended.getX(),
            blended.getY(),
            blended.getRotation().getDegrees()
        };
        NetworkTableInstance.getDefault()
            .getTable("/AdvantageScope/Vision")
            .getEntry("InterpolatedPose")
            .setDoubleArray(blended2d);

        restoreCameraSettings();
    }

    /** @return Last computed robot gobal position with blue side left as origin. */
    public Pose2d getEstimatedPose() {
        return lastEstimatedPose;
    }

    /** @return True if Limelight sees an AprilTag. */
    public boolean hasValidAprilTarget() {
        return limelightTable.getEntry("tv").getDouble(0) >= 1.0;
    }

    private void switchCameraMode(CameraMode mode) {
        limelightTable.getEntry("camMode").setNumber(mode.value);
    }

    private void switchPipeline(Pipeline pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline.value);
    }

    /**
     * Retrieve raw object detector results.
     * @return Array of [classId, xNorm, yNorm, area]
     */
    public Double[][] getDetectorResults() {
        // Force detector pipeline and vision mode
        switchPipeline(Pipeline.OBJECT_DETECTION);
        switchCameraMode(CameraMode.VISION);

        RawDetection[] raw = LimelightHelpers.getRawDetections(Constants.Vision.LIMELIGHT_TABLE_NAME);
        lastTargetCount = raw.length;
        if (raw.length == 0) {
            restoreCameraSettings();
            return new Double[0][0];
        }

        Double[][] results = new Double[raw.length][4];
        for (int i = 0; i < raw.length; i++) {
            var d = raw[i];
            results[i] = new Double[]{
                (double) d.classId,
                d.txnc,
                d.tync,
                d.ta
            };
        }

        restoreCameraSettings();
        return results;
    }

    /** @return Number of targets seen by the last detector call. */
    public int getLastTargetCount() {
        return lastTargetCount;
    }

    /**
     * Gets the "best" coral detection (largest area) from Limelight AI detector.
     * Temporarily switches to OBJECT_DETECTION pipeline, reads detections, then restores settings.
     * @return RawDetection or null if none found
     */
    public LimelightHelpers.RawDetection getBestCoralDetection() {
        // Remember current settings
        var savedMode = currentCamMode;
        var savedPipe = currentPipeline;

        try {
            // Force OD pipeline to fetch detections
            switchPipeline(Pipeline.OBJECT_DETECTION);
            switchCameraMode(CameraMode.VISION);

            LimelightHelpers.RawDetection[] dets =
                LimelightHelpers.getRawDetections(Constants.Vision.LIMELIGHT_TABLE_NAME);
            if (dets == null || dets.length == 0) return null;

            // Filter to coral class and pick max area
            return Arrays.stream(dets)
                .filter(d -> d.classId == Constants.Vision.coralClassId)
                .max(Comparator.comparingDouble(d -> d.ta))
                .orElse(null);
        } finally {
            // Restore user-selected vision settings
            currentCamMode = savedMode;
            currentPipeline = savedPipe;
            restoreCameraSettings();
        }
    }

    // -----------------------------------------------------------------------
    // Private Helpers
    // -----------------------------------------------------------------------
    
    private void restoreCameraSettings() {
        switchPipeline(currentPipeline);
        switchCameraMode(currentCamMode);
    }

    // Decide if the vision pose is close enough to odometry/gyro to trust
    private boolean isPoseCloseToOdom(Pose2d visionPose) {
        Pose2d odomPose = drive.getPose();
        double transErr = odomPose.getTranslation().getDistance(visionPose.getTranslation());
        double yawErrDeg = Math.abs(visionPose.getRotation().minus(odomPose.getRotation()).getDegrees());
        if (yawErrDeg > 180.0) yawErrDeg = 360.0 - yawErrDeg;

        SmartDashboard.putNumber("Vision/OdomTransErr", transErr);
        SmartDashboard.putNumber("Vision/OdomYawErrDeg", yawErrDeg);

        return transErr <= Constants.Vision.visionMaxTranslationErrorMeters
            && yawErrDeg <= Constants.Vision.visionMaxYawErrorDeg;
    }

    @Override
    public void periodic() {
        updatePose();
    }

    @Override
    public void simulationPeriodic() {
        updatePose();
    }
}
