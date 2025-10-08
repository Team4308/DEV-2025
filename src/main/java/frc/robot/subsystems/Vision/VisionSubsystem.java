package frc.robot.subsystems.Vision;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers.RawDetection;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends SubsystemBase {
    // Hardware & Estimator
    private final DifferentialDriveKinematics kinematics;
    public final AHRS gyro;
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

    private double prevLeftDistance = 0.0;
    private double prevRightDistance = 0.0;

    private double nextSimVisionTime = 0.0;
    private double lastSimVisionTimestamp = -1.0;

    private final DriveSystem drive;

    /**enum for limelight network table camera mode configuration. CameraMode.VISION or CameraMode.DRIVER*/
    public enum CameraMode { VISION(0), DRIVER(1); public final int value; CameraMode(int v) { value = v; }}

    /**enum for limelight network table pipeline configuration. Pipeline.APRIL_TAGS or Pipeline.OBJECT_DETECTION*/
    public enum Pipeline { APRIL_TAGS(0), OBJECT_DETECTION(1); public final int value; Pipeline(int v) { value = v; }}



    /** Constructor: configure sensors, estimator, and Limelight */
    public VisionSubsystem(DriveSystem drive) {
        // Initialize kinematics and sensors
        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidthMeters);
        gyro = DriveSystem.imu;
        this.drive = drive;
        this.leftEncoder = drive.leftEncoder;
        this.rightEncoder = drive.rightEncoder;

        // Initialize previous encoder distances
        prevLeftDistance = leftEncoder.getDistance();
        prevRightDistance = rightEncoder.getDistance();

        // Configure pose estimator with noise from constants
        poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance(),
            new Pose2d(),
            VecBuilder.fill(
                Constants.Vision.stateStdDevPosMeters,
                Constants.Vision.stateStdDevPosMeters,
                Units.degreesToRadians(Constants.Vision.stateStdDevThetaDeg)
            ),
            VecBuilder.fill(
                Constants.Vision.visionStdDevPosMeters,
                Constants.Vision.visionStdDevPosMeters,
                Units.degreesToRadians(Constants.Vision.visionStdDevThetaDeg)
            )
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
     * Update the pose estimator with odometry and vision data if valid. 
     * Computes robot global position with blue side left as origin.
     * Should be called periodically.
    */
    public void updatePose() {
        // --- ODOMETRY UPDATE ---
        double leftDist = leftEncoder.getDistance();
        double rightDist = rightEncoder.getDistance();
        poseEstimator.update(
            gyro.getRotation2d(),
            leftDist,
            rightDist
        );
        prevLeftDistance = leftDist;
        prevRightDistance = rightDist;

        // --- VISION UPDATE ---
        switchPipeline(Pipeline.APRIL_TAGS);
        switchCameraMode(CameraMode.VISION);

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            SmartDashboard.putNumberArray("LL Raw BotPose", botpose);
            double tv = limelightTable.getEntry("tv").getDouble(0.0);

            if (tv >= 1.0 && botpose.length >= 7) {
                Pose2d simPose = new Pose2d(
                    botpose[0], // x (m)
                    botpose[1], // y (m)
                    Rotation2d.fromDegrees(botpose[5]) // yaw (deg)
                );
                double latencySec = botpose[6] / 1000.0;
                double ts = Timer.getFPGATimestamp() - latencySec;

                if (ts > lastSimVisionTimestamp) {
                    poseEstimator.addVisionMeasurement(
                        simPose,
                        ts,
                        VecBuilder.fill(
                            Constants.Vision.simVisionStdDevPosMeters,
                            Constants.Vision.simVisionStdDevPosMeters,
                            Units.degreesToRadians(Constants.Vision.simVisionStdDevThetaDeg)
                        )
                    );
                    lastSimVisionTimestamp = ts;
                }
            }
        } else {
            double currentRotation = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
            LimelightHelpers.SetRobotOrientation("limelight", currentRotation, 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate visionResult = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.LIMELIGHT_TABLE_NAME);

            double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            SmartDashboard.putNumberArray("LL Raw BotPose", botpose);

            if (visionResult != null) {
                // Debug: Output vision pose and timestamp
                SmartDashboard.putNumber("LL Vision X", visionResult.pose.getX());
                SmartDashboard.putNumber("LL Vision Y", visionResult.pose.getY());
                SmartDashboard.putNumber("LL Vision Rot", visionResult.pose.getRotation().getDegrees());
                SmartDashboard.putNumber("LL Vision Timestamp", visionResult.timestampSeconds);
                SmartDashboard.putNumber("LL Vision TagCount", visionResult.tagCount);

                // Only use vision if pose is not zero and at least one tag is detected
                boolean validVision = hasValidAprilTarget()
                    && Math.abs(gyro.getRate()) < Constants.Vision.maxGyroRate
                    && visionResult.tagCount > 0
                    && !(visionResult.pose.getX() == 0 && visionResult.pose.getY() == 0);

                if (validVision) {
                    double measStd = visionResult.avgTagDist / Constants.Vision.idealDetectionRange;
                    poseEstimator.addVisionMeasurement(
                        visionResult.pose,
                        visionResult.timestampSeconds,
                        VecBuilder.fill(measStd, measStd, Double.MAX_VALUE)
                    );
                    onNewVisionPose(visionResult.pose);
                }
            } else {
                // Only show "No vision result" if not in simulation
                if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
                    SmartDashboard.putString("LL Vision Status", "No vision result");
                }
            }
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

    private void onNewVisionPose(Pose2d estimatedPose) {
        double ts = Timer.getFPGATimestamp();
        drive.addVisionMeasurement(estimatedPose, ts);
    }

    @Override
    public void periodic() {
        // Always update pose estimator with odometry and vision
        updatePose();

        // Publish pose for AdvantageScope and SmartDashboard
        double[] poseArr = {
            lastEstimatedPose.getX(),
            lastEstimatedPose.getY(),
            lastEstimatedPose.getRotation().getDegrees()
        };
        NetworkTableInstance.getDefault()
            .getTable("/AdvantageScope/Vision")
            .getEntry("EstimatedPose")
            .setDoubleArray(poseArr);
        SmartDashboard.putNumberArray("EstimatedPose", poseArr);

        // Optionally publish an "ideal" pose as a ghost
        double[] idealArr = {
            Constants.Vision.idealDetectionRange, 0, 0
        };
        NetworkTableInstance.getDefault()
            .getTable("/AdvantageScope/Vision")
            .getEntry("IdealPose")
            .setDoubleArray(idealArr);
        SmartDashboard.putNumberArray("IdealPose", idealArr);
    }

    @Override
    public void simulationPeriodic() {
        // Simulate odometry update
        double leftDist = leftEncoder.getDistance();
        double rightDist = rightEncoder.getDistance();
        poseEstimator.update(
            gyro.getRotation2d(),
            leftDist,
            rightDist
        );
        prevLeftDistance = leftDist;
        prevRightDistance = rightDist;

        // Publish a simulated Limelight pose to NT at configured rate
        double now = Timer.getFPGATimestamp();
        if (now >= nextSimVisionTime) {
            Pose2d simVisionPose = poseEstimator.getEstimatedPosition();
            pushSimLimelightPose(simVisionPose, Constants.Vision.simLatencyMs);
            nextSimVisionTime = now + Constants.Vision.simUpdatePeriodSec;
        }

        lastEstimatedPose = poseEstimator.getEstimatedPosition();
    }

    private void pushSimLimelightPose(Pose2d pose, double latencyMs) {
        // Populate NT entries to emulate Limelight outputs
        double[] arr = new double[] {
            pose.getX(),                       // X (m)
            pose.getY(),                       // Y (m)
            0.0,                               // Z (m)
            0.0,                               // roll (deg)
            0.0,                               // pitch (deg)
            pose.getRotation().getDegrees(),   // yaw (deg)
            latencyMs                          // latency (ms)
        };
        limelightTable.getEntry("tv").setDouble(1.0);
        limelightTable.getEntry("botpose_wpiblue").setDoubleArray(arr);
        limelightTable.getEntry("pipeline").setNumber(Pipeline.APRIL_TAGS.value);
        limelightTable.getEntry("camMode").setNumber(CameraMode.VISION.value);
        SmartDashboard.putString("LL Vision Status", "Simulated vision injected");
    }
}