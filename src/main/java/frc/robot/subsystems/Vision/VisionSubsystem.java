package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.LimelightHelpers;
import java.util.function.Supplier;
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

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.0)); // Example track width
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private final Encoder m_leftEncoder = new Encoder(0, 1); // filler encoder ports
    private final Encoder m_rightEncoder = new Encoder(2, 3); // filler encoder ports
    private final DifferentialDrivePoseEstimator m_poseEstimator = 
    new DifferentialDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );
    private Pose2d posEstimate;
    private double confidence;

    public VisionSubsystem(Supplier<Pose2d> currentPos) {
        setCamMode(0); // Vision processing mode
        LimelightHelpers.setLEDMode_ForceOn("limelight");
        LimelightHelpers.setCameraPose_RobotSpace("limelight",  // meters and degrees rn
        0.5,    // Forward offset (meters)
        0.0,    // Side offset (meters)
        0.5,    // Height offset (meters)
        0.0,    // Roll (degrees)
        30.0,   // Pitch (degrees)
        0.0     // Yaw (degrees)
        );
    }

    public void updateEstimatedPose() {
        Boolean rejectUpdate = false;  
        LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        if(!hasValidTarget()){rejectUpdate = true;}
        if(!rejectUpdate) {
            m_poseEstimator.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds,
                VecBuilder.fill(confidence, confidence, 9999999)
                );
        }
        posEstimate = m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedPose(){
        return posEstimate;
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

    @Override
    public void periodic() {
        updateEstimatedPose();
    }
}