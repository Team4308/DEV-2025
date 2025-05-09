package frc.robot.subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers.RawDetection;


public class VisionSubsystem extends SubsystemBase {
    // Hardware & Estimator
    private final DifferentialDriveKinematics kinematics;
    private final ADXRS450_Gyro gyro;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final DifferentialDrivePoseEstimator poseEstimator;

    // NetworkTables
    private final NetworkTable limelightTable;
    
    // State
    private CameraMode currentCamMode;
    private Pipeline currentPipeline;
    private Pose2d lastEstimatedPose;
    private int lastTargetCount;

    /**enum for limelight network table camera mode configuration. CameraMode.VISION or CameraMode.DRIVER*/
    public enum CameraMode { VISION(0), DRIVER(1); public final int value; CameraMode(int v) { value = v; }}

    /**enum for limelight network table pipeline configuration. Pipeline.APRIL_TAGS or Pipeline.OBJECT_DETECTION*/
    public enum Pipeline { APRIL_TAGS(0), OBJECT_DETECTION(1); public final int value; Pipeline(int v) { value = v; }}



    /** Constructor: configure sensors, estimator, and Limelight */
    public VisionSubsystem() {
        // Initialize kinematics and sensors
        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidthMeters);
        gyro = new ADXRS450_Gyro();
        leftEncoder = new Encoder(Constants.DriveConstants.leftEncoderChannelA, Constants.DriveConstants.leftEncoderChannelB);
        rightEncoder = new Encoder(Constants.DriveConstants.rightEncoderChannelA, Constants.DriveConstants.rightEncoderChannelB);
        
        // Configure pose estimator
        poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),   // state std devs
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(20))    // vision meas std devs
        );

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
     * Update the pose estimator with vision data if valid. 
     * computes robot gobal position with blue side left as origin.
     * Should be called periodically.
    */
    public void updatePose() {
        // Temporarily force Apriltag pipeline & vision mode for measurement
        switchPipeline(Pipeline.APRIL_TAGS);
        switchCameraMode(CameraMode.VISION);

        double currentRotation = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation("limelight", currentRotation, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate visionResult = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.LIMELIGHT_TABLE_NAME);

        // Validate result
        if (hasValidAprilTarget() && Math.abs(gyro.getRate()) < Constants.Vision.maxGyroRate) {
            double measStd = visionResult.avgTagDist / Constants.Vision.idealDetectionRange;
            poseEstimator.addVisionMeasurement(
                visionResult.pose,
                visionResult.timestampSeconds,
                VecBuilder.fill(measStd, measStd, Double.MAX_VALUE)
            );
        }

        lastEstimatedPose = poseEstimator.getEstimatedPosition();

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

    // -----------------------------------------------------------------------
    // Private Helpers
    // -----------------------------------------------------------------------
    private void restoreCameraSettings() {
        switchPipeline(currentPipeline);
        switchCameraMode(currentCamMode);
    }
}