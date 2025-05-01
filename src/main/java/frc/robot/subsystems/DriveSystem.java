package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class DriveSystem extends SubsystemBase {
    // Follower Motors
    public static TalonSRX leaderLeft, leaderRight;
    // Follower Motors
        
    private final TalonSRX followerLeft1, followerRight1, followerLeft2, followerRight2;
    private AHRS imu = new AHRS(NavXComType.kI2C);

    public DriveSystem() {
        // Setup Motors

        leaderLeft = new TalonSRX(Constants.Mapping.Drive.Left);
        leaderRight = new TalonSRX(Constants.Mapping.Drive.Right);
        followerLeft1 = new TalonSRX(Constants.Mapping.Drive.Left1);
        followerRight1 = new TalonSRX(Constants.Mapping.Drive.Right1);
        followerLeft2 = new TalonSRX(Constants.Mapping.Drive.Left2);
        followerRight2 = new TalonSRX(Constants.Mapping.Drive.Right1);
        followerLeft1.set(ControlMode.Follower, leaderLeft.getDeviceID());
        followerLeft2.set(ControlMode.Follower, leaderLeft.getDeviceID());
        followerRight1.set(ControlMode.Follower, leaderRight.getDeviceID());
        followerRight2.set(ControlMode.Follower, leaderRight.getDeviceID());
    }

    public void setPowerPercent(double left, double right) {
        leaderLeft.set(TalonSRXControlMode.PercentOutput, left);
        leaderRight.set(TalonSRXControlMode.PercentOutput, right);
    }

    public void StopControllers() {
        leaderLeft.set(TalonSRXControlMode.PercentOutput, 0);
        leaderRight.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void ToggleBrake() {
        leaderLeft.setNeutralMode(NeutralMode.Brake);
        leaderRight.setNeutralMode(NeutralMode.Brake);
        
    }

    public void ToggleCoast() {
        leaderLeft.setNeutralMode(NeutralMode.Coast);
        leaderRight.setNeutralMode(NeutralMode.Coast);
        
    }
    
}

