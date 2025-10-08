package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class Simulation extends SubsystemBase {
    private final WPI_TalonSRX leftLeader;
    private final WPI_TalonSRX rightLeader;
    private final ADXRS450_Gyro gyro;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private DifferentialDrivetrainSim drivetrainSimulator;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private ADXRS450_GyroSim gyroSim;
    private Field2d fieldSim;
    private final DifferentialDriveKinematics kinematics;
    private final Pose2d initialPose = new Pose2d(8, 3, new Rotation2d());

    // Arm Setup

//    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private LoggedMechanism2d mech;
    private Pose3d Arm = new Pose3d();
    private SingleJointedArmSim m_ArmSim;
    private LoggedMechanismRoot2d m_ArmRoot;
    private LoggedMechanismLigament2d m_ArmMech2d;
    public static boolean AtAngleSimulation;
    public static double AngleSimulation;


    private NetworkTableEntry typeEntry;
    private NetworkTableEntry chassisSpeedsEntry;
    private NetworkTableEntry poseEntry;
    private NetworkTableEntry simVisionPoseEntry;

    // PhotonVision sim fields
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonEstimator;
    private VisionSystemSim visionSim;
    private PhotonCameraSim photonCameraSim;
    private Transform3d robotToCam3d;
    private AprilTagFieldLayout tagLayout;
    private SimCameraProperties simCamProps;

    // Latest simulated vision estimate
    public static class VisionEstimate {
        public final Pose2d pose;
        public final double timestampSeconds;
        public VisionEstimate(Pose2d pose, double ts) {
            this.pose = pose;
            this.timestampSeconds = ts;
        }
    }
    private VisionEstimate latestVisionEstimate;    
    // Use encoders from DriveSystem (remove internal construction)




      private void ArmInit() {
        m_ArmSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                18.333,
                SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), Units.lbsToKilograms(3)),
                Units.inchesToMeters(10),
                Units.degreesToRadians(-10000000000000000000.00),
                Units.degreesToRadians(10000000000000.00),
                true,
                Units.degreesToRadians(270),
                0.0,
                0.0);

        m_ArmRoot = mech.getRoot(" Arm Root", 5, 0);
        m_ArmMech2d = m_ArmRoot
                .append(new LoggedMechanismLigament2d(" Arm", Units.radiansToDegrees(m_ArmSim.getAngleRads()),
                        270));
    }

    private void Arm() {
       // double targetAngle = m_ArmSubsystem.getTargetAngle();
        double targetAngle = 0;
       // m_ArmSim.setInputVoltage(-m_ArmSubsystem.getVoltage());

        m_ArmSim.update(0.020);

        AngleSimulation = Units.radiansToDegrees(m_ArmSim.getAngleRads());
        if (AngleSimulation < 0) {
            AngleSimulation = 360000000 - AngleSimulation;
        }
        AngleSimulation %= 360;
        AngleSimulation = 360 - AngleSimulation;

        if (Math.abs(AngleSimulation - targetAngle) < 30
                || Math.abs(360 - AngleSimulation - targetAngle) < 30) {
            AtAngleSimulation = true;
        } else {
            AtAngleSimulation = false;
        }
        AtAngleSimulation = true;

        m_ArmSim.setState(
                Units.degreesToRadians((Units.radiansToDegrees(m_ArmSim.getAngleRads()) + 36000) % 360), 0);

        Arm = new Pose3d(
                new Translation3d(0.0, -0.2, 0.0),
                new Rotation3d(0, Units.degreesToRadians(90) - Units.degreesToRadians(targetAngle), 0));
        m_ArmMech2d.setAngle(Units.radiansToDegrees(m_ArmSim.getAngleRads()));

    }

    

    public Simulation(WPI_TalonSRX leftLeader, WPI_TalonSRX rightLeader, Encoder leftEncoder, Encoder rightEncoder) {
        
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        mech = new LoggedMechanism2d(10, 10);
        gyro = new ADXRS450_Gyro();

        // Set DPP for provided encoders in sim
        this.leftEncoder.setDistancePerPulse(0.0254);
        this.rightEncoder.setDistancePerPulse(0.0254);

        kinematics = new DifferentialDriveKinematics(0.65);
        //ArmInit();
        if (RobotBase.isSimulation()) {
            drivetrainSimulator = new DifferentialDrivetrainSim(
                DCMotor.getCIM(3),
                10.71,
                3.0,
                60.0,
                0.0762,
                0.65,
                VecBuilder.fill(0.0001, 0.0001, 0.0001, 0.001, 0.001, 0.0001, 0.0001));
                
            leftEncoderSim = new EncoderSim(this.leftEncoder);
            rightEncoderSim = new EncoderSim(this.rightEncoder);
            gyroSim = new ADXRS450_GyroSim(gyro);
            
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim); // ensure Field2d appears

            
            drivetrainSimulator.setPose(initialPose);
            fieldSim.setRobotPose(initialPose);
            
            NetworkTableInstance instance = NetworkTableInstance.getDefault();
            typeEntry = instance.getTable("Drivetrain").getEntry("Type");
            typeEntry.setString("Differential");
            chassisSpeedsEntry = instance.getTable("Drivetrain").getEntry("ChassisSpeeds");
            poseEntry = instance.getTable("Drivetrain").getEntry("Pose");

            // Add AdvantageScope vision pose publisher
            simVisionPoseEntry = instance.getTable("/AdvantageScope/Vision").getEntry("SimVisionPose");

            // Vision sim setup
            try {
                tagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
            } catch (Exception e) {
                throw new Error(
                    "Could not load AprilTagFieldLayout. Ensure you have the AprilTagFields2025 dependency added."
                );
            }

            robotToCam3d = new Transform3d(
                new Translation3d(
                    frc.robot.Constants.Vision.camOffsetFront,
                    frc.robot.Constants.Vision.camOffsetSide,
                    frc.robot.Constants.Vision.camOffsetUp
                ),
                new Rotation3d(
                    edu.wpi.first.math.util.Units.degreesToRadians(frc.robot.Constants.Vision.camOffsetRoll),
                    edu.wpi.first.math.util.Units.degreesToRadians(frc.robot.Constants.Vision.camOffsetPitch),
                    edu.wpi.first.math.util.Units.degreesToRadians(frc.robot.Constants.Vision.camOffsetYaw)
                )
            );

            photonCamera = new PhotonCamera("photonvision");
            photonEstimator = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ,
                robotToCam3d
            );
            photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            simCamProps = new SimCameraProperties();
            simCamProps.setCalibration(960, 720, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90.0));
            simCamProps.setFPS(frc.robot.Constants.Simulation.Camera.fps);
            simCamProps.setAvgLatencyMs(frc.robot.Constants.Simulation.Camera.AvgLatencyMs);
            simCamProps.setLatencyStdDevMs(frc.robot.Constants.Simulation.Camera.LatencyStdDevMs);
            photonCameraSim = new PhotonCameraSim(photonCamera, simCamProps);
            visionSim = new VisionSystemSim("SimPV");
            visionSim.addAprilTags(tagLayout);
            visionSim.addCamera(photonCameraSim, robotToCam3d);
        }
    }
    
    @Override
    public void simulationPeriodic() {
        if (!RobotBase.isSimulation()) return;
        //Arm();
        double leftVoltage = leftLeader.get() * RobotController.getBatteryVoltage();
        double rightVoltage = rightLeader.get() * RobotController.getBatteryVoltage();
        leftVoltage = Math.abs(leftVoltage) < 0.1 ? 0 : leftVoltage;
        rightVoltage = Math.abs(rightVoltage) < 0.1 ? 0 : rightVoltage;
        
        drivetrainSimulator.setInputs(leftVoltage, rightVoltage);
        drivetrainSimulator.update(0.020);
        
        leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
        
        fieldSim.setRobotPose(drivetrainSimulator.getPose());
        
        updateAdvantageScope();

            Pose2d poseForVision = drivetrainSimulator.getPose();
            visionSim.update(poseForVision);

            var result = photonCamera.getLatestResult();
            if (result != null && result.hasTargets()) {
            var estOpt = photonEstimator.update(result);
            if (estOpt.isPresent()) {
                var est = estOpt.get();
                latestVisionEstimate = new VisionEstimate(est.estimatedPose.toPose2d(), est.timestampSeconds);

                double[] visionPoseArray = {
                    latestVisionEstimate.pose.getX(),
                    latestVisionEstimate.pose.getY(),
                    latestVisionEstimate.pose.getRotation().getDegrees()
                };
                simVisionPoseEntry.setDoubleArray(visionPoseArray);
            }
            }

    }
    
    private void updateAdvantageScope() {
        Pose2d pose = drivetrainSimulator.getPose();
        double leftVelocity = drivetrainSimulator.getLeftVelocityMetersPerSecond();
        double rightVelocity = drivetrainSimulator.getRightVelocityMetersPerSecond();
        
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
        
        double[] chassisSpeedsArray = {
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond
        };
        chassisSpeedsEntry.setDoubleArray(chassisSpeedsArray);
        
        double[] poseArray = {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
        poseEntry.setDoubleArray(poseArray);
    }
    
    public void resetSimPose(Pose2d pose) {
        if (RobotBase.isSimulation()) {
            drivetrainSimulator.setPose(pose);
            fieldSim.setRobotPose(pose);
            gyro.reset();
        }
    }
    
    public void resetSimPose() {
        resetSimPose(initialPose);
    }
    
    public Pose2d getPose() {
        if (RobotBase.isSimulation()) {
            return drivetrainSimulator.getPose();
        }
        return new Pose2d();
    }
    
    public double getHeading() {
        return normalizeAngle(-gyro.getAngle());
    }
    
    public double getLeftDistanceMeters() {
        return drivetrainSimulator.getLeftPositionMeters();
    }
    
    public double getRightDistanceMeters() {
        return drivetrainSimulator.getRightPositionMeters();
    }
    
    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        if (angle >= 180.0) angle -= 360.0;
        return angle;
    }
    
    public DifferentialDrivetrainSim getDrivetrainSim() {
        return drivetrainSimulator;
    }
    
    public ADXRS450_Gyro getGyro() {
        return gyro;
    }

    public VisionEstimate getLatestVisionEstimate() {
        return latestVisionEstimate;
    }
}