package frc.robot.subsystems.Vision;

import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers;
import pabeles.concurrency.IntOperatorTask.Max;

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
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.0)); // filler track width
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();   // filler gyro
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
    private double ideal_detection_range = Constants.Vision.ideal_detection_range;

    public VisionSubsystem(Supplier<Pose2d> currentPos) {
        setCamMode(0); // Vision processing mode
        LimelightHelpers.setLEDMode_ForceOn("limelight");
        LimelightHelpers.setCameraPose_RobotSpace("limelight",  // meters and degrees
        Constants.Vision.cam_offset_front,   
        Constants.Vision.cam_offset_side,   
        Constants.Vision.cam_offset_up,   
        Constants.Vision.cam_offset_roll,   
        Constants.Vision.cam_offset_pitch, 
        Constants.Vision.cam_offset_yaw   
        );
    }

    public void updateEstimatedPose() {     // gobal position with blue side right as origin
        Boolean rejectUpdate = false;  
        LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        if(!hasValidTarget() || Math.abs(m_gyro.getRate()) > 360){rejectUpdate = true;}
        if(!rejectUpdate) {
            //confidence = Math.max(1, ideal_detection_range/limelightMeasurement.avgTagDist);
            confidence = 0.7;   
            m_poseEstimator.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds,
                VecBuilder.fill(confidence, confidence, 9999999)
                );
        }
        posEstimate = m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedPose(){ // gobal position with blue side right as origin
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