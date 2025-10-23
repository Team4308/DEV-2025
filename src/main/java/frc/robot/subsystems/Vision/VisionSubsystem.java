package frc.robot.subsystems.Vision;

import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;
import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsystem extends SubsystemBase {
    // Hardware & Estimator
    public final AHRS gyro;

    // NetworkTables
    
    // State
    private Pose2d lastEstimatedPose;
    private Pose2d lastVisionPose; 
    private int lastTargetCount;

    private double lastSimVisionTimestamp = -1.0;

    private final DriveSystem drive;

    private final PhotonCamera photonCam0 = new PhotonCamera("photonvision"); 
    private final PhotonCamera photonCam1 = new PhotonCamera("photonvision2");  
    private final Transform3d robotToCam0 = new Transform3d(new Translation3d(0.3, 0.20, 0.45), new Rotation3d(0, 0, 0));
    private final Transform3d robotToCam1 = new Transform3d(new Translation3d(-0.2, -0.20, 0.38), new Rotation3d(0, Math.toRadians(10), Math.PI));
    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final PhotonPoseEstimator photonEstimator0 =
        new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam0);
    private final PhotonPoseEstimator photonEstimator1 =
        new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam1);



    /** Constructor: configure sensors, estimator, and Limelight */
    public VisionSubsystem(DriveSystem drive) {
        // Initialize kinematics and sensors
        gyro = DriveSystem.imu;
        this.drive = drive;

        // general setup

        lastEstimatedPose = new Pose2d();
        lastTargetCount = 0;

        // Configure Photon estimators
        photonEstimator0.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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

        // PhotonVision fusion 
        var photonTable = NetworkTableInstance.getDefault().getTable("/AdvantageScope/Photon");

        PhotonPipelineResult res0 = photonCam0.getLatestResult();
        PhotonPipelineResult res1 = photonCam1.getLatestResult();

        EstimatedRobotPose est0 = estimateFrom(res0, photonEstimator0);
        EstimatedRobotPose est1 = estimateFrom(res1, photonEstimator1);

        publishPhoton(photonTable, "Cam0Pose", est0);
        publishPhoton(photonTable, "Cam1Pose", est1);

        EstimatedRobotPose bestEst = chooseBest(est0, res0, est1, res1);
        if (bestEst != null) {
            Pose2d photonPose = bestEst.estimatedPose.toPose2d();
            if (isPoseCloseToOdom(photonPose)) {
                drive.addVisionMeasurement(photonPose, bestEst.timestampSeconds);
                lastVisionPose = photonPose;
                photonTable.getEntry("FusedPose").setDoubleArray(new double[] {
                    photonPose.getX(), photonPose.getY(), photonPose.getRotation().getDegrees()
                });
                photonTable.getEntry("Timestamp").setDouble(bestEst.timestampSeconds);
            } else {
                photonTable.getEntry("FusedPose").setDoubleArray(new double[0]);
            }
        } else {
            photonTable.getEntry("FusedPose").setDoubleArray(new double[0]);
        }

        // Blend output (using lastVisionPose if set)
        double alpha = (lastVisionPose != null) ? 0.75 : 0.25;
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
        NetworkTableInstance.getDefault()
            .getTable("/AdvantageScope/Vision")
            .getEntry("InterpolatedPose")
            .setDoubleArray(new double[] { blended.getX(), blended.getY(), blended.getRotation().getDegrees() });

    }





    /** @return Number of targets seen by the last detector call. */
    public int getLastTargetCount() {
        return lastTargetCount;
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

    // Photon helpers used in updatePose()
    private EstimatedRobotPose estimateFrom(PhotonPipelineResult res, PhotonPoseEstimator estimator) {
        if (res == null || !res.hasTargets()) return null;
        var opt = estimator.update(res);
        return opt.isPresent() ? opt.get() : null;
    }

    private EstimatedRobotPose chooseBest(EstimatedRobotPose e0, PhotonPipelineResult r0,
                                          EstimatedRobotPose e1, PhotonPipelineResult r1) {
        if (e0 == null && e1 == null) return null;
        if (e0 != null && e1 == null) return e0;
        if (e1 != null && e0 == null) return e1;

        int c0 = r0 != null ? r0.getTargets().size() : 0;
        int c1 = r1 != null ? r1.getTargets().size() : 0;
        if (c0 != c1) return c0 > c1 ? e0 : e1;

        double a0 = (r0 != null && r0.getBestTarget() != null) ? r0.getBestTarget().getPoseAmbiguity() : 1.0;
        double a1 = (r1 != null && r1.getBestTarget() != null) ? r1.getBestTarget().getPoseAmbiguity() : 1.0;
        return a0 <= a1 ? e0 : e1;
    }

    private void publishPhoton(NetworkTable table, String key, EstimatedRobotPose est) {
        if (table == null) return;
        if (est == null) {
            table.getEntry(key).setDoubleArray(new double[0]);
        } else {
            Pose2d p = est.estimatedPose.toPose2d();
            table.getEntry(key).setDoubleArray(new double[] { p.getX(), p.getY(), p.getRotation().getDegrees() });
        }
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
