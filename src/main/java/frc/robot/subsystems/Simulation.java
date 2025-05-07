package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

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
    private final Pose2d initialPose = new Pose2d(2, 2, new Rotation2d());
    
    private NetworkTableEntry typeEntry;
    private NetworkTableEntry chassisSpeedsEntry;
    private NetworkTableEntry poseEntry;
    
    public Simulation(WPI_TalonSRX leftLeader, WPI_TalonSRX rightLeader) {
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;
        
        gyro = new ADXRS450_Gyro();
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);
        
        leftEncoder.setDistancePerPulse(0.0254);
        rightEncoder.setDistancePerPulse(0.0254);
        
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
                
            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            gyroSim = new ADXRS450_GyroSim(gyro);
            
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim);
            
            drivetrainSimulator.setPose(initialPose);
            fieldSim.setRobotPose(initialPose);
            
            NetworkTableInstance instance = NetworkTableInstance.getDefault();
            typeEntry = instance.getTable("Drivetrain").getEntry("Type");
            typeEntry.setString("Differential");
            chassisSpeedsEntry = instance.getTable("Drivetrain").getEntry("ChassisSpeeds");
            poseEntry = instance.getTable("Drivetrain").getEntry("Pose");
        }
    }
    
    @Override
    public void simulationPeriodic() {
        if (!RobotBase.isSimulation()) return;
        
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
        
        SmartDashboard.putNumber("Left Voltage", leftVoltage);
        SmartDashboard.putNumber("Right Voltage", rightVoltage);
        SmartDashboard.putNumber("Heading", drivetrainSimulator.getHeading().getDegrees());
        SmartDashboard.putNumber("Left Distance", leftEncoderSim.getDistance());
        SmartDashboard.putNumber("Right Distance", rightEncoderSim.getDistance());
        
        updateAdvantageScope();
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
}