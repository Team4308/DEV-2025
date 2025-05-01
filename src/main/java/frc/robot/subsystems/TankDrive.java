package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DigitalInput;

public class TankDrive extends SubsystemBase {
    // Follower Motors
    public final TalonFX leaderLeft, leaderRight;

    // Follower Motors
        
    private final TalonFX followerLeft1, followerRight1, followerLeft2, followerRight2;
    
    // Controllers
    private ArrayList<TalonFX> controllersFX = new ArrayList<TalonFX>();
    
    public static ADIS16470_IMU gyro = new ADIS16470_IMU(); // figure out gyro later
    public static DifferentialDriveOdometry odometry;
    public TankDrive() {

        // Setup Motors
        leaderLeft = new TalonFX(Constants.Mapping.Drive.frontLeft);
        leaderRight = new TalonFX(Constants.Mapping.Drive.frontRight);
        followerLeft1 = new TalonFX(Constants.Mapping.Drive.backLeft);
        followerRight1 = new TalonFX(Constants.Mapping.Drive.backRight);
        followerLeft2 = new TalonFX(Constants.Mapping.Drive.backLeft);
        followerRight2 = new TalonFX(Constants.Mapping.Drive.backRight);

    }

}

