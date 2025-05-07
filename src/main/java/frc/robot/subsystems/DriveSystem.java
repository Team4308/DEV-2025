package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class DriveSystem extends SubsystemBase {
    public static WPI_TalonSRX leaderLeft, leaderRight;
    private final WPI_TalonSRX followerLeft1, followerRight1, followerLeft2, followerRight2;
    private AHRS imu = new AHRS(NavXComType.kI2C);
    private DifferentialDrive diffDrive;
    private DifferentialDriveOdometry m_odometry;
    private Simulation simulation;

    public DriveSystem() {
        leaderLeft = new WPI_TalonSRX(Constants.Mapping.Drive.Left);
        leaderRight = new WPI_TalonSRX(Constants.Mapping.Drive.Right);
        followerLeft1 = new WPI_TalonSRX(Constants.Mapping.Drive.Left1);
        followerRight1 = new WPI_TalonSRX(Constants.Mapping.Drive.Right1);
        followerLeft2 = new WPI_TalonSRX(Constants.Mapping.Drive.Left2);
        followerRight2 = new WPI_TalonSRX(Constants.Mapping.Drive.Right2); 
        
        followerLeft1.set(ControlMode.Follower, leaderLeft.getDeviceID());
        followerLeft2.set(ControlMode.Follower, leaderLeft.getDeviceID());
        followerRight1.set(ControlMode.Follower, leaderRight.getDeviceID());
        followerRight2.set(ControlMode.Follower, leaderRight.getDeviceID());
        
        setNeutralMode(NeutralMode.Brake);
        
        diffDrive = new DifferentialDrive(leaderLeft, leaderRight);
        diffDrive.setSafetyEnabled(false);
        
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
        
        m_odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()), 
            0.0, 0.0);
        
        if (RobotBase.isSimulation()) {
            simulation = new Simulation(leaderLeft, leaderRight);
        }
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(getHeading()),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );
        
    }

    public void setPowerPercent(double left, double right) {
        left = Math.max(-1.0, Math.min(1.0, left));
        right = Math.max(-1.0, Math.min(1.0, right));
        
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
        return m_odometry.getPoseMeters();
    }

    public double getHeading() {
        if (RobotBase.isSimulation() && simulation != null) {
            return simulation.getHeading();
        } else if (imu != null) {
            return normalizeAngle(-imu.getAngle());
        }
        return 0.0;
    }
    
    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        if (angle >= 180.0) angle -= 360.0;
        return angle;
    }
    
    public double getLeftDistanceMeters() {
        if (RobotBase.isSimulation() && simulation != null) {
            return simulation.getLeftDistanceMeters();
        }
        return 0.0;
    }
    
    public double getRightDistanceMeters() {
        if (RobotBase.isSimulation() && simulation != null) {
            return simulation.getRightDistanceMeters();
        }
        return 0.0;
    }

    public void resetSimPose() {
        if (RobotBase.isSimulation() && simulation != null) {
            simulation.resetSimPose();
        }
    }
    
    public void resetHeading() {
        if (RobotBase.isSimulation() && simulation != null) {
            simulation.getGyro().reset();
        } else if (imu != null) {
            imu.reset();
        }
    }
}

