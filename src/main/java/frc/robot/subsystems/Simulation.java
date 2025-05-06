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

public class Simulation extends SubsystemBase {
    // The motors
    private final WPI_TalonSRX leftLeader;
    private final WPI_TalonSRX rightLeader;
    
    // The gyro sensor
    private final ADXRS450_Gyro gyro;
    
    // The encoders
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    
    // Simulation classes
    private DifferentialDrivetrainSim drivetrainSimulator;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private ADXRS450_GyroSim gyroSim;
    private Field2d fieldSim;
    
    public Simulation(WPI_TalonSRX leftLeader, WPI_TalonSRX rightLeader) {
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;
        
        // Initialize sensors
        gyro = new ADXRS450_Gyro();
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);
        
        // Configure encoders
        leftEncoder.setDistancePerPulse(0.0254); // 1 inch per pulse
        rightEncoder.setDistancePerPulse(0.0254);
        
        if (RobotBase.isSimulation()) {
            drivetrainSimulator = new DifferentialDrivetrainSim(
                DCMotor.getCIM(3),
                8.45,
                2.0,
                50.0,
                0.0762,
                0.559,
                null);
                
            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            gyroSim = new ADXRS450_GyroSim(gyro);
            
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim);
            
            Pose2d initialPose = new Pose2d(2, 2, new Rotation2d());
            drivetrainSimulator.setPose(initialPose);
            fieldSim.setRobotPose(initialPose);
        }
    }
    
    @Override
    public void simulationPeriodic() {
        if (RobotBase.isSimulation()) {
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
        }
    }
    
    public void resetSimPose(Pose2d pose) {
        if (RobotBase.isSimulation()) {
            drivetrainSimulator.setPose(pose);
            fieldSim.setRobotPose(pose);
        }
    }
    
    public Pose2d getPose() {
        if (RobotBase.isSimulation()) {
            return drivetrainSimulator.getPose();
        }
        return new Pose2d();
    }
    
    public DifferentialDrivetrainSim getDrivetrainSim() {
        return drivetrainSimulator;
    }
}