package frc.robot.subsystems;


import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
public class DriveSystem extends SubsystemBase {
    public static WPI_TalonSRX leaderLeft, leaderRight;
    private final WPI_TalonSRX followerLeft1, followerRight1, followerLeft2, followerRight2;
    public Encoder leftEncoder, rightEncoder;
    public static AHRS imu = new AHRS(NavXComType.kI2C);
    private DifferentialDrive diffDrive;
    private DifferentialDriveOdometry m_odometry;
    public static Simulation simulation;
    private RobotConfig config;
  private Pose2d targetPose = new Pose2d();

  public Pose2d nearestPoseToLeftReef = new Pose2d();
  public Pose2d nearestPoseToRightReef = new Pose2d();
  public Pose2d nearestPoseToAlgaeRemove = new Pose2d();
  public Pose2d nearestPoseToFarCoralStation = new Pose2d();
  public Pose2d nearestPoseToNearCoralStation = new Pose2d();



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
        leftEncoder = new Encoder(Constants.DriveConstants.leftEncoderChannelA, Constants.DriveConstants.leftEncoderChannelB);
        rightEncoder = new Encoder(Constants.DriveConstants.rightEncoderChannelA, Constants.DriveConstants.rightEncoderChannelB);


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
            // Pass encoders into Simulation so EncoderSim updates these same encoders
            simulation = new Simulation(leaderLeft, leaderRight, leftEncoder, rightEncoder);
        }

        try {
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.out.println("Error loading robot config: " + e.getMessage());
        }
        AutoBuilder.configure(
            this::getPose, 
            pose -> resetHeading(), 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPLTVController(0.02), 
            config, 
            () -> {

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this 
        );
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

        return leftEncoder.getDistance();
    }
    
    public double getRightDistanceMeters() {
        if (RobotBase.isSimulation() && simulation != null) {
            return simulation.getRightDistanceMeters();
        }
        return rightEncoder.getDistance();
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
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
      }
    

    private edu.wpi.first.math.kinematics.ChassisSpeeds getRobotRelativeSpeeds() {
        return new edu.wpi.first.math.kinematics.ChassisSpeeds();
    }

    private void driveRobotRelative(edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
        double forward = speeds.vxMetersPerSecond;
        double rotation = speeds.omegaRadiansPerSecond;

        double maxSpeed = 3.0; // meters per sec
        double maxAngular = Math.PI; // rad per sec

        double forwardPercent = Math.max(-1.0, Math.min(1.0, forward / maxSpeed));
        double rotationPercent = Math.max(-1.0, Math.min(1.0, rotation / maxAngular));

        diffDrive.arcadeDrive(forwardPercent, rotationPercent, false);
        }


         public Pose2d getClosestLeftReefPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.REEF.RED_LEFT_REEF_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.REEF.BLUE_LEFT_REEF_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestRightReefPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.REEF.RED_RIGHT_REEF_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.REEF.BLUE_RIGHT_REEF_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestAlgaeRemovePose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.ALGAE.RED_ALGAE_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.ALGAE.BLUE_ALGAE_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestFarCoralStationPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.RED_FAR_STATION_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.BLUE_FAR_STATION_POSES);
    }
    return nearestPose;
  }

  public Pose2d getClosestNearCoralStationPose() {
    Pose2d nearestPose = new Pose2d();
    if (isRedAlliance()) {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.RED_NEAR_STATION_POSES);
    } else {
      nearestPose = getPose().nearest(FieldLayout.CORAL_STATION.BLUE_NEAR_STATION_POSES);
    }
    return nearestPose;
  }

  public Command updateClosestReefPoses() {
    return this.runOnce(() -> {
      nearestPoseToLeftReef = getClosestLeftReefPose();
      nearestPoseToRightReef = getClosestRightReefPose();
    });
  }

  public Command updateClosestAlgaePose() {
    return this.runOnce(() -> {
      nearestPoseToAlgaeRemove = getClosestAlgaeRemovePose();
    });
  }

  public Command updateClosestStationPose() {
    return this.runOnce(() -> {
      nearestPoseToFarCoralStation = getClosestFarCoralStationPose();
      nearestPoseToNearCoralStation = getClosestNearCoralStationPose();
    });
  }

  public boolean isTranslationAligned() {
    Translation2d currentTranslation2d = getPose().getTranslation();
    Translation2d targetTranslation2d = targetPose.getTranslation();
    if (currentTranslation2d.getDistance(targetTranslation2d) < Constants.DriveConstants.HeadingTolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isHeadingAligned() {
    double currentHeading = getPose().getRotation().getDegrees();
    double targetHeading = targetPose.getRotation().getDegrees();
    if ((Math.abs(currentHeading - targetHeading) < Constants.DriveConstants.HeadingTolerance)
        || (Math.abs(currentHeading + targetHeading) % 360 < Constants.DriveConstants.HeadingTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isAligned() {
    if (isTranslationAligned() && isHeadingAligned()) {
      return true;
    } else {
      return false;
    }
  }

        
        public Command driveToPose(Pose2d pose) {
        targetPose = pose;
        PathConstraints constraints = new PathConstraints(3.0, 3.0, Math.PI, 3.0);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(getPose(), pose);
        PathPlannerPath path = new PathPlannerPath(

            waypoints,
            constraints,
            new IdealStartingState(0.0, Rotation2d.fromDegrees(getHeading())),
            new GoalEndState(0.0, pose.getRotation())
        );
        return AutoBuilder.followPath(path)
            .andThen(() -> StopControllers());
        }

        public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
        return run(() -> setPowerPercent(speedInMetersPerSecond / 3.0, speedInMetersPerSecond / 3.0))
            .until(() -> getLeftDistanceMeters() >= distanceInMeters || getRightDistanceMeters() >= distanceInMeters)
            .andThen(() -> StopControllers());
        }

        public static Simulation getSimulation() {
        return simulation;
    }

}
