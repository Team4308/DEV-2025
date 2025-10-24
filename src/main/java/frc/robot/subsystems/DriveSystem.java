package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class DriveSystem extends SubsystemBase {
  // Motors
  public static WPI_TalonSRX leaderLeft, leaderRight;
  private final WPI_TalonSRX followerLeftRear, followerRightRear;
  // Sensors
  private final CANcoder leftEncoder, rightEncoder;
  public static AHRS imu = new AHRS(NavXComType.kI2C);

  private DifferentialDrive diffDrive;
  public DifferentialDriveOdometry m_odometry;
  public static Simulation simulation;
  private RobotConfig config;
  private Pose2d targetPose = new Pose2d();

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.DriveConstants.trackWidthMeters);
  private DifferentialDrivePoseEstimator poseEstimator;

  private Orchestra orchestra;

  private double leftTargetMeters = 0.0;
  private double rightTargetMeters = 0.0;
  private double lastTs = Timer.getFPGATimestamp();

  public DriveSystem() {
    leaderLeft = new WPI_TalonSRX(Constants.DriveConstants.Left_Front);
    leaderRight = new WPI_TalonSRX(Constants.DriveConstants.Right_Front);
    followerLeftRear = new WPI_TalonSRX(Constants.DriveConstants.Left_Back);
    followerRightRear = new WPI_TalonSRX(Constants.DriveConstants.Right_Back);

    followerLeftRear.set(ControlMode.Follower, leaderLeft.getDeviceID());
    followerRightRear.set(ControlMode.Follower, leaderRight.getDeviceID());

    leaderLeft.setInverted(Constants.DriveConstants.Left_Front_Inverted);
    leaderRight.setInverted(Constants.DriveConstants.Right_Front_Inverted);

    followerLeftRear.setInverted(Constants.DriveConstants.Left_Back_Inverted);
    followerRightRear.setInverted(Constants.DriveConstants.Right_Back_Inverted);

    leftEncoder = new CANcoder(Constants.DriveConstants.leftEncoder);
    rightEncoder = new CANcoder(Constants.DriveConstants.rightEncoder);

    configMotors(leaderLeft, Constants.DriveConstants.leftEncoder);
    configMotors(leaderRight, Constants.DriveConstants.rightEncoder);

    leftTargetMeters = getLeftDistanceMeters();
    rightTargetMeters = getRightDistanceMeters();
    lastTs = Timer.getFPGATimestamp();

    orchestra = new Orchestra();

    setNeutralMode(NeutralMode.Brake);

    diffDrive = new DifferentialDrive(leaderLeft, leaderRight);
    diffDrive.setSafetyEnabled(false);

    leaderLeft.configNominalOutputForward(Constants.DriveConstants.nominalOutputForward);
    leaderLeft.configNominalOutputReverse(Constants.DriveConstants.nominalOutputReverse);
    leaderLeft.configPeakOutputForward(Constants.DriveConstants.peakOutputForward);
    leaderLeft.configPeakOutputReverse(Constants.DriveConstants.peakOutputReverse);
    leaderRight.configNominalOutputForward(Constants.DriveConstants.nominalOutputForward);
    leaderRight.configNominalOutputReverse(Constants.DriveConstants.nominalOutputReverse);
    leaderRight.configPeakOutputForward(Constants.DriveConstants.peakOutputForward);
    leaderRight.configPeakOutputReverse(Constants.DriveConstants.peakOutputReverse);

    if (RobotBase.isSimulation()) {
      simulation = new Simulation(new Encoder(0, 1), new Encoder(2, 3));
      CommandScheduler.getInstance().registerSubsystem(simulation);
    }

    m_odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(getHeading()),
        0.0, 0.0);

    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        Rotation2d.fromDegrees(getHeading()),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        new Pose2d());

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.out.println("Error loading robot config: " + e.getMessage());
    }
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPLTVController(Constants.DriveConstants.ltvUpdatePeriodSec),
        config,
        () -> {

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  private void configMotors(WPI_TalonSRX talon, int cancoderId) {

    // Magic Motion config
    int to = 100;
    talon.configRemoteFeedbackFilter(cancoderId, RemoteSensorSource.CANCoder, 0, to);
    talon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, to);
    talon.setSensorPhase(false);

    talon.config_kP(0, Constants.DriveConstants.mm_kP, to);
    talon.config_kI(0, Constants.DriveConstants.mm_kI, to);
    talon.config_kD(0, Constants.DriveConstants.mm_kD, to);
    talon.config_kF(0, Constants.DriveConstants.mm_kF, to);
  }

  @Override
  public void periodic() {

    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

    poseEstimator.update(
        Rotation2d.fromDegrees(getHeading()),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

    // Meters travelled
    SmartDashboard.putNumber("Drive/LeftMeters", getLeftDistanceMeters());
    SmartDashboard.putNumber("Drive/RightMeters", getRightDistanceMeters());
    SmartDashboard.putNumber("Drive/AvgMeters", 0.5 * (getLeftDistanceMeters() + getRightDistanceMeters()));

    SmartDashboard.putNumber("Drive/RightEncoderMeters", getRightDistanceMeters());
    SmartDashboard.putNumber("Drive/LeftEncoderMeters", getLeftDistanceMeters());

  }

  public void setPowerPercent(double left, double right) {
    left = Math.max(-1.0, Math.min(1.0, left));
    right = Math.max(-1.0, Math.min(1.0, right));

    leaderLeft.set(TalonSRXControlMode.PercentOutput, left);
    leaderRight.set(TalonSRXControlMode.PercentOutput, right);

    if (RobotBase.isSimulation() && simulation != null) {
      simulation.setOpenLoopPercents(left, right);
    }
  }

  public void StopControllers() {
    leaderLeft.set(TalonSRXControlMode.PercentOutput, 0);
    leaderRight.set(TalonSRXControlMode.PercentOutput, 0);
  }

  private void setNeutralMode(NeutralMode mode) {
    leaderLeft.setNeutralMode(mode);
    leaderRight.setNeutralMode(mode);
    followerLeftRear.setNeutralMode(mode);
    followerRightRear.setNeutralMode(mode);
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
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        pose);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
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
    if (angle < 0)
      angle += 360.0;
    if (angle >= 180.0)
      angle -= 360.0;
    return angle;
  }

  public double getLeftDistanceMeters() {
    if (RobotBase.isSimulation() && simulation != null) {
      return simulation.getLeftDistanceMeters();
    }
    double rot = leftEncoder.getPosition().getValueAsDouble(); // rotations
    return rot * Constants.DriveConstants.metersPerEncoderRotation;
  }

  public double getRightDistanceMeters() {
    if (RobotBase.isSimulation() && simulation != null) {
      return simulation.getRightDistanceMeters();
    }
    double rot = rightEncoder.getPosition().getValueAsDouble(); // rotations
    return rot * Constants.DriveConstants.metersPerEncoderRotation;
  }

  public void resetSimPose() {
    if (RobotBase.isSimulation() && simulation != null) {
      simulation.resetSimPose();
      resetPose(new Pose2d());
    }
  }

  public void resetHeading() {
    if (RobotBase.isSimulation() && simulation != null) {
      simulation.resetHeading();
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

    double maxSpeed = Constants.DriveConstants.maxLinearSpeedMps;
    double maxAngular = Constants.DriveConstants.maxAngularSpeedRadPerSec;

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
    PathConstraints constraints = new PathConstraints(
        Constants.DriveConstants.pathMaxVelMps,
        Constants.DriveConstants.pathMaxAccelMps2,
        Constants.DriveConstants.pathMaxAngularVelRadPerSec,
        Constants.DriveConstants.pathMaxAngularAccelRadPerSec2);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(getPose(), pose);
    PathPlannerPath path = new PathPlannerPath(

        waypoints,
        constraints,
        new IdealStartingState(0.0, Rotation2d.fromDegrees(getHeading())),
        new GoalEndState(0.0, pose.getRotation()));
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

  public boolean playSong(String chrpFilename) {
    var status = orchestra.loadMusic(chrpFilename);
    if (status.isOK()) {
      orchestra.play();
      return true;
    }
    System.out.println("Orchestra load failed: " + status.toString());
    return false;
  }

  public void stopSong() {
    if (orchestra != null) {
      orchestra.stop();
    }
  }

  public void pauseSong() {
    if (orchestra != null) {
      orchestra.pause();
    }
  }

  public boolean isSongPlaying() {
    return orchestra != null && orchestra.isPlaying();
  }

  // Motion Magic with Arcade Drive
  public void arcadeDriveClosedLoop(double xSpeed, double zRotation, boolean useMM) {
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(0.0, now - lastTs);
    lastTs = now;

    useMM = false;

    if (useMM) {
      double v = xSpeed * Constants.DriveConstants.maxLinearSpeedMps;
      double omega = zRotation * Constants.DriveConstants.maxAngularSpeedRadPerSec;

      double halfTrack = Constants.DriveConstants.trackWidthMeters / 2.0;
      double leftDelta = (v - omega * halfTrack) * dt;
      double rightDelta = (v + omega * halfTrack) * dt;

      leftTargetMeters += leftDelta;
      rightTargetMeters += rightDelta;

      int leftTicks = (int) Math.round(leftTargetMeters * Constants.DriveConstants.ticksPerMeter);
      int rightTicks = (int) Math.round(rightTargetMeters * Constants.DriveConstants.ticksPerMeter);

      leaderLeft.set(TalonSRXControlMode.MotionMagic, leftTicks);
      leaderRight.set(TalonSRXControlMode.MotionMagic, rightTicks);
    } else {
      double left = xSpeed + zRotation;
      double right = xSpeed - zRotation;
      setPowerPercent(left, right);
    }
  }

}
