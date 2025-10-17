package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.RobotController;
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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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

import frc.robot.Constants;

public class Simulation extends SubsystemBase {
    private final WPI_TalonSRX leftLeader;
    private final WPI_TalonSRX rightLeader;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private DifferentialDrivetrainSim drivetrainSimulator;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private Field2d fieldSim;
    private final DifferentialDriveKinematics kinematics;
    private final Pose2d initialPose = new Pose2d(8, 3, new Rotation2d());
    private DifferentialDriveOdometry simOdometry;
    private NetworkTableEntry encoderOdomPoseEntry;

    // Arm Setup
    private LoggedMechanism2d mech;
    private Pose3d Arm = new Pose3d();
    private SingleJointedArmSim m_ArmSim;
    private LoggedMechanismRoot2d m_ArmRoot;
    private LoggedMechanismLigament2d m_ArmMech2d;
    public static boolean AtAngleSimulation;
    public static double AngleSimulation;
    private double armInputVoltage = 0.0;
    private double armTargetAngleDeg = 0.0;

    private NetworkTableEntry typeEntry;
    private NetworkTableEntry chassisSpeedsEntry;
    private NetworkTableEntry poseEntry;
    private NetworkTableEntry simVisionPoseEntry;
    private NetworkTableEntry visionPose2dEntry;
    private NetworkTableEntry interpPose2dEntry;
    private NetworkTableEntry armAngleDegEntry;
    private NetworkTableEntry armTargetDegEntry;
    private NetworkTableEntry armVoltageEntry;

    // PhotonVision sim 
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonEstimator;
    private VisionSystemSim visionSim;
    private PhotonCameraSim photonCameraSim;
    private Transform3d robotToCam3d;
    private AprilTagFieldLayout tagLayout;
    private SimCameraProperties simCamProps;

    //  vision estimate
    public static class VisionEstimate {
        public final Pose2d pose;
        public final double timestampSeconds;
        public VisionEstimate(Pose2d pose, double ts) {
            this.pose = pose;
            this.timestampSeconds = ts;
        }
    }
    private VisionEstimate latestVisionEstimate;    
    private double headingDeg = 0.0;

    private void ArmInit() {
        m_ArmSim = new SingleJointedArmSim(
            DCMotor.getNEO(Constants.Simulation.arm.motorCount),
            Constants.Simulation.arm.gearRatio,
            SingleJointedArmSim.estimateMOI(Constants.Simulation.arm.armLengthMeters, Constants.Simulation.arm.armMassKg),
            Constants.Simulation.arm.armLengthMeters,
            Units.degreesToRadians(Constants.Simulation.arm.minAngleDeg),
            Units.degreesToRadians(Constants.Simulation.arm.maxAngleDeg),
            Constants.Simulation.arm.simulateGravity,
            Units.degreesToRadians(Constants.Simulation.arm.armOffsetDeg),
            0.0,
            0.0
        );

        m_ArmRoot = mech.getRoot("Arm Root", 5, 0);
        m_ArmMech2d = m_ArmRoot.append(
            new LoggedMechanismLigament2d("Arm", Constants.Simulation.arm.armLengthMeters, 0)
        );
    }

    private void Arm() {
        m_ArmSim.setInputVoltage(armInputVoltage);
        m_ArmSim.update(Constants.Simulation.arm.simLoopPeriodSec);
        double rawDeg = Units.radiansToDegrees(m_ArmSim.getAngleRads());
        double wrapped = ((rawDeg % 360.0) + 360.0) % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        AngleSimulation = Math.abs(wrapped);
        AtAngleSimulation = Math.abs(AngleSimulation - armTargetAngleDeg) <= Constants.Simulation.arm.atAngleToleranceDeg;

        if (m_ArmMech2d != null) {
            m_ArmMech2d.setAngle(AngleSimulation);
            m_ArmMech2d.setLength(Constants.Simulation.arm.armLengthMeters);
        }

        if (armAngleDegEntry != null) armAngleDegEntry.setDouble(AngleSimulation);
        if (armTargetDegEntry != null) armTargetDegEntry.setDouble(armTargetAngleDeg);
        if (armVoltageEntry != null) armVoltageEntry.setDouble(armInputVoltage);
    }

    public Simulation(WPI_TalonSRX leftLeader, WPI_TalonSRX rightLeader, Encoder leftEncoder, Encoder rightEncoder) {
        
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        mech = new LoggedMechanism2d(10, 10);

        this.leftEncoder.setDistancePerPulse(0.0254);
        this.rightEncoder.setDistancePerPulse(0.0254);

        kinematics = new DifferentialDriveKinematics(0.65);
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
            
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim); 

            
            drivetrainSimulator.setPose(initialPose);
            fieldSim.setRobotPose(initialPose);
            
            NetworkTableInstance instance = NetworkTableInstance.getDefault();
            typeEntry = instance.getTable("Drivetrain").getEntry("Type");
            typeEntry.setString("Differential");
            chassisSpeedsEntry = instance.getTable("Drivetrain").getEntry("ChassisSpeeds");
            poseEntry = instance.getTable("Drivetrain").getEntry("Pose");
            simVisionPoseEntry = instance.getTable("/AdvantageScope/Vision").getEntry("SimVisionPose");
            visionPose2dEntry = instance.getTable("/AdvantageScope/Vision").getEntry("VisionPose");
            interpPose2dEntry = instance.getTable("/AdvantageScope/Vision").getEntry("InterpolatedPose");

            
            var armTable = instance.getTable("/AdvantageScope/Arm");
            armAngleDegEntry = armTable.getEntry("AngleDeg");
            armTargetDegEntry = armTable.getEntry("TargetAngleDeg");
            armVoltageEntry = armTable.getEntry("Voltage");

            SmartDashboard.putData("Arm 3D", mech);
            ArmInit();

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

            encoderOdomPoseEntry = instance.getTable("/AdvantageScope/Drive").getEntry("EncoderOdomPose");
            // Initialize sim odometry from current heading and starting distances
            simOdometry = new DifferentialDriveOdometry(
                Rotation2d.fromDegrees(headingDeg), 0.0, 0.0, initialPose
            );
        }
    }
    
    @Override
    public void simulationPeriodic() {
        if (!RobotBase.isSimulation()) return;
        // Arm sim tick
        Arm();

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
        setSimHeadingDegrees(-drivetrainSimulator.getHeading().getDegrees());
        
        fieldSim.setRobotPose(drivetrainSimulator.getPose());
        
        updateAdvantageScope();

        // Update encoder+gyro odometry and publish
        var odomPose = simOdometry.update(
            Rotation2d.fromDegrees(headingDeg),
            drivetrainSimulator.getLeftPositionMeters(),
            drivetrainSimulator.getRightPositionMeters()
        );
        if (encoderOdomPoseEntry != null) {
            encoderOdomPoseEntry.setDoubleArray(new double[] {
                odomPose.getX(), odomPose.getY(), odomPose.getRotation().getDegrees()
            });
        }

            Pose2d poseForVision = drivetrainSimulator.getPose();
            visionSim.update(poseForVision);

            var result = photonCamera.getLatestResult();
            NetworkTable limelight = NetworkTableInstance.getDefault().getTable(frc.robot.Constants.Vision.LIMELIGHT_TABLE_NAME);
            if (result != null) {
                int tagCount = result.hasTargets() ? result.getTargets().size() : 0;

                if (result.hasTargets()) {
                    var estOpt = photonEstimator.update(result);
                    if (estOpt.isPresent()) {
                        var est = estOpt.get();
                        latestVisionEstimate = new VisionEstimate(est.estimatedPose.toPose2d(), est.timestampSeconds);

                        tagCount = result.getTargets().size();
                        
                        double[] visionPose2d = {
                            latestVisionEstimate.pose.getX(),
                            latestVisionEstimate.pose.getY(),
                            latestVisionEstimate.pose.getRotation().getDegrees()
                        };
                        simVisionPoseEntry.setDoubleArray(visionPose2d);
                        visionPose2dEntry.setDoubleArray(visionPose2d);

                        Pose2d odomPoseForVision = drivetrainSimulator.getPose();
                        Pose2d interp = new Pose2d(
                            odomPoseForVision.getTranslation().interpolate(latestVisionEstimate.pose.getTranslation(), 0.5),
                            odomPoseForVision.getRotation().interpolate(latestVisionEstimate.pose.getRotation(), 0.5)
                        );
                        double[] interpPose2d = {
                            interp.getX(),
                            interp.getY(),
                            interp.getRotation().getDegrees()
                        };
                        interpPose2dEntry.setDoubleArray(interpPose2d);

                        double latencyMs = frc.robot.Constants.Vision.simLatencyMs;
                        double[] botpose = new double[] {
                            latestVisionEstimate.pose.getX(),
                            latestVisionEstimate.pose.getY(),
                            0.0,
                            0.0,
                            0.0,
                            latestVisionEstimate.pose.getRotation().getDegrees(),
                            latencyMs
                        };
                        limelight.getEntry("tv").setDouble(1.0);
                        limelight.getEntry("botpose_wpiblue").setDoubleArray(botpose);
                        limelight.getEntry("pipeline").setNumber(0);
                        limelight.getEntry("camMode").setNumber(0);
                        limelight.getEntry("tc").setNumber(tagCount);
                        NetworkTableInstance.getDefault()
                            .getTable("/AdvantageScope/Vision")
                            .getEntry("TagCount")
                            .setDouble(tagCount);
                    } else {
                        // Targets but no pose estimate: no valid target for LL consumers
                        limelight.getEntry("tv").setDouble(0.0);
                        limelight.getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
                        limelight.getEntry("tc").setNumber(tagCount);
                        NetworkTableInstance.getDefault()
                            .getTable("/AdvantageScope/Vision")
                            .getEntry("TagCount")
                            .setDouble(tagCount);
                    }
                } else {
                    // No targets: clear LL outputs
                    limelight.getEntry("tv").setDouble(0.0);
                    limelight.getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
                    limelight.getEntry("tc").setNumber(0);
                    NetworkTableInstance.getDefault()
                        .getTable("/AdvantageScope/Vision")
                        .getEntry("TagCount")
                        .setDouble(0);
                }
            } else {
                // No result at all: clear LL outputs
                limelight.getEntry("tv").setDouble(0.0);
                limelight.getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
                limelight.getEntry("tc").setNumber(0);
                NetworkTableInstance.getDefault()
                    .getTable("/AdvantageScope/Vision")
                    .getEntry("TagCount")
                    .setDouble(0);
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
            resetHeading();
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
        return headingDeg;
    }
    
    public double getLeftDistanceMeters() {
        return drivetrainSimulator.getLeftPositionMeters();
    }
    
    public double getRightDistanceMeters() {
        return drivetrainSimulator.getRightPositionMeters();
    }
    

    
    public DifferentialDrivetrainSim getDrivetrainSim() {
        return drivetrainSimulator;
    }

    public VisionEstimate getLatestVisionEstimate() {
        return latestVisionEstimate;
    }

    private void setSimHeadingDegrees(double heading) {
        this.headingDeg = heading;
    }

    public void resetHeading() {
        headingDeg = 0.0;
    }
    public void setArmVoltage(double volts) {
        armInputVoltage = volts;
    }
    public void setArmTargetAngleDeg(double targetDeg) {
        armTargetAngleDeg = targetDeg;
    }
    public double getArmAngleDeg() {
        return AngleSimulation;
    }
}