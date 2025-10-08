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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;


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

     // Sim 


    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonEstimator;
    private VisionSystemSim visionSim;
    private Transform3d robotToCam3d;
    private AprilTagFieldLayout tagLayout;
    private SimCameraProperties simCamProps;

    /**enum for limelight network table camera mode configuration. CameraMode.VISION or CameraMode.DRIVER*/
    public enum CameraMode { VISION(0), DRIVER(1); public final int value; CameraMode(int v) { value = v; }}

    /**enum for limelight network table pipeline configuration. Pipeline.APRIL_TAGS or Pipeline.OBJECT_DETECTION*/
    public enum Pipeline { APRIL_TAGS(0), OBJECT_DETECTION(1); public final int value; Pipeline(int v) { value = v; }}



    /** Constructor: configure sensors, estimator, and Limelight */
    public VisionSubsystem(Encoder leftEncoder, Encoder rightEncoder) {
        // Initialize kinematics and sensors
        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidthMeters);
        gyro = DriveSystem.imu;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    
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
        
        if (RobotBase.isSimulation()) {
            try {
                // Adjust field if needed
                tagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
            } catch (Exception e) {
            }

            robotToCam3d = new Transform3d(
                new Translation3d(
                    Constants.Vision.camOffsetFront,
                    Constants.Vision.camOffsetSide,
                    Constants.Vision.camOffsetUp
                ),
                new Rotation3d(
                    Units.degreesToRadians(Constants.Vision.camOffsetRoll),
                    Units.degreesToRadians(Constants.Vision.camOffsetPitch),
                    Units.degreesToRadians(Constants.Vision.camOffsetYaw)
                )
            );

            photonCamera = new PhotonCamera("photonvision");
            photonEstimator = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.AVERAGE_BEST_TARGETS,
                robotToCam3d
            );

            PhotonCameraSim photonCameraSim = new PhotonCameraSim(photonCamera);
            simCamProps = new SimCameraProperties();
            simCamProps.setCalibration(960, 720, new Rotation2d(Units.degreesToRadians(90.0)));
            simCamProps.setFPS(Constants.Simulation.Camera.fps);
            simCamProps.setAvgLatencyMs(Constants.Simulation.Camera.AvgLatencyMs);
            simCamProps.setLatencyStdDevMs(Constants.Simulation.Camera.LatencyStdDevMs);    

            visionSim = new VisionSystemSim("SimPV");
            visionSim.addAprilTags(tagLayout);
            visionSim.addCamera(photonCameraSim, robotToCam3d);
        }
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


        // --- VISION UPDATE ---
        switchPipeline(Pipeline.APRIL_TAGS);
        switchCameraMode(CameraMode.VISION);

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            var result = photonCamera != null ? photonCamera.getLatestResult() : null;
            if (result != null && result.hasTargets()) {
                var estOpt = photonEstimator.update(result);
                if (estOpt.isPresent()) {
                    var est = estOpt.get();
                    poseEstimator.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(5))
                    );
                    SmartDashboard.putString("LL Vision Status", "Photon sim vision used");
                }
            }
        } else {
            double currentRotation = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
            LimelightHelpers.SetRobotOrientation("limelight", currentRotation, 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate visionResult = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.LIMELIGHT_TABLE_NAME);

            double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            SmartDashboard.putNumberArray("LL Raw BotPose", botpose);

            if (visionResult != null) {
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
                }
            } else {
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

        if (visionSim != null) {
            Pose2d pose2d = poseEstimator.getEstimatedPosition();
            visionSim.update(pose2d);
        }

        lastEstimatedPose = poseEstimator.getEstimatedPosition();
    }
}