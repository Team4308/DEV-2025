package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Constants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class DriveSystem extends SubsystemBase {
    // Leader Motors
    public static WPI_TalonSRX leaderLeft, leaderRight;
    // Follower Motors
    private final WPI_TalonSRX followerLeft1, followerRight1, followerLeft2, followerRight2;
    private AHRS imu = new AHRS(NavXComType.kI2C);
    
    // Simulation instance
    private Simulation simulation;

    // Add field for visualization
    private Field2d m_fieldSim;

    public DriveSystem() {
        // Setup Motors
        leaderLeft = new WPI_TalonSRX(Constants.Mapping.Drive.Left);
        leaderRight = new WPI_TalonSRX(Constants.Mapping.Drive.Right);
        followerLeft1 = new WPI_TalonSRX(Constants.Mapping.Drive.Left1);
        followerRight1 = new WPI_TalonSRX(Constants.Mapping.Drive.Right1);
        followerLeft2 = new WPI_TalonSRX(Constants.Mapping.Drive.Left2);
        followerRight2 = new WPI_TalonSRX(Constants.Mapping.Drive.Right2); 
        
        // Configure followers
        followerLeft1.set(ControlMode.Follower, leaderLeft.getDeviceID());
        followerLeft2.set(ControlMode.Follower, leaderLeft.getDeviceID());
        followerRight1.set(ControlMode.Follower, leaderRight.getDeviceID());
        followerRight2.set(ControlMode.Follower, leaderRight.getDeviceID());
        
        setNeutralMode(NeutralMode.Brake);
        
        if (RobotBase.isSimulation()) {
            simulation = new Simulation(leaderLeft, leaderRight);
            m_fieldSim = new Field2d();
            SmartDashboard.putData("Field", m_fieldSim);
        }

        leaderLeft.configNominalOutputForward(0);
        leaderLeft.configNominalOutputReverse(0);
        leaderLeft.configPeakOutputForward(1.0);
        leaderLeft.configPeakOutputReverse(-1.0);
        leaderRight.configNominalOutputForward(0);
        leaderRight.configNominalOutputReverse(0);
        leaderRight.configPeakOutputForward(1.0);
        leaderRight.configPeakOutputReverse(-1.0);

        leaderRight.setInverted(true);
        followerRight1.setInverted(true);
        followerRight2.setInverted(true);
    }

    @Override
    public void periodic() {
        // Publish data to SmartDashboard
        SmartDashboard.putNumber("Left Motor Output", leaderLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Motor Output", leaderRight.getMotorOutputPercent());
        
        if (imu != null) {
            SmartDashboard.putNumber("Robot Heading", imu.getAngle());
        }
        
        if (RobotBase.isSimulation() && simulation != null) {
            Pose2d pose = simulation.getPose();
            SmartDashboard.putNumber("Robot X Position", pose.getX());
            SmartDashboard.putNumber("Robot Y Position", pose.getY());
            SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
            
            // Update field visualization with current robot pose
            if (m_fieldSim != null) {
                m_fieldSim.setRobotPose(getPose());
            }
        }
    }
    
    @Override
    public void simulationPeriodic() {
        if (simulation != null) {
            simulation.simulationPeriodic();
        }
    }

    public void setPowerPercent(double left, double right) {
        leaderLeft.set(TalonSRXControlMode.PercentOutput, left);
        leaderRight.set(TalonSRXControlMode.PercentOutput, right);
    }

    public void StopControllers() {
        leaderLeft.set(TalonSRXControlMode.PercentOutput, 0);
        leaderRight.set(TalonSRXControlMode.PercentOutput, 0);
    }

    private void setNeutralMode(NeutralMode mode) {
        leaderLeft.setNeutralMode(mode);
        leaderRight.setNeutralMode(mode);
        followerLeft1.setNeutralMode(mode);
        followerLeft2.setNeutralMode(mode);
        followerRight1.setNeutralMode(mode);
        followerRight2.setNeutralMode(mode);
    }

    public void ToggleBrake() {
        setNeutralMode(NeutralMode.Brake);
    }

    public void ToggleCoast() {
        setNeutralMode(NeutralMode.Coast);
    }
    
    public AHRS getIMU() {
        return imu;
    }


    public Pose2d getPose() {
        if (RobotBase.isSimulation() && simulation != null) {
            return simulation.getPose();
        }
        return new Pose2d();
    }
}

