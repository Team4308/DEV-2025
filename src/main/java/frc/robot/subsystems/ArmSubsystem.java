package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;

    private final ProfiledPIDController controller = new ProfiledPIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(
            ArmConstants.MAX_VEL_DEG_PER_S,
            ArmConstants.MAX_ACC_DEG_PER_S2
        )
    );

    private final ArmFeedforward ff = new ArmFeedforward(
        ArmConstants.kS,
        ArmConstants.kG,
        ArmConstants.kV
    );

    private double targetAngle = ArmConstants.RESTING_ANGLE;

    public ArmSubsystem() {

        // Spark Max Config
        armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.encoder.positionConversionFactor(ArmConstants.DEG_PER_ENCODER_UNIT);
        config.encoder.velocityConversionFactor(ArmConstants.DEG_PER_ENCODER_UNIT / 60.0);
        armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        encoder = armMotor.getEncoder();
        // Starting Deg
        double initialRawDeg = (RobotBase.isSimulation() && DriveSystem.getSimulation() != null)
            ? DriveSystem.getSimulation().getArmAngleDeg()
            : encoder.getPosition();
        double initAngleDeg = RobotBase.isSimulation() ? initialRawDeg : (initialRawDeg - ArmConstants.ZERO_OFFSET_DEG);
        targetAngle = initAngleDeg;
        controller.reset(initAngleDeg);
        controller.setGoal(targetAngle);
        controller.setTolerance(1.0);

        // Simulation update
        if (RobotBase.isSimulation() && DriveSystem.getSimulation() != null) {
            DriveSystem.getSimulation().setArmTargetAngleDeg(targetAngle);
        }
    }

    public void moveToScoringPosition() { updateTargetAngleDeg(ArmConstants.SCORING_ANGLE); }
    public void moveToRestingPosition() { updateTargetAngleDeg(ArmConstants.RESTING_ANGLE); }
    public void moveToFeederPosition() { updateTargetAngleDeg(ArmConstants.FEEDER_ANGLE); }
    public void moveToGroundIntakePosition() { updateTargetAngleDeg(ArmConstants.GROUND_ANGLE); }

    public int getCurrentAngle() { return (int) encoder.getPosition(); }
    public int getTargetAngle() { return (int) targetAngle; }
    public double getVoltage() { return armMotor.getAppliedOutput(); }

    private void applyMotorOutput(double volts) {
        double clamped = Math.max(-12.0, Math.min(12.0, volts));
        armMotor.setVoltage(clamped);
        if (RobotBase.isSimulation() && DriveSystem.getSimulation() != null) {
            DriveSystem.getSimulation().setArmVoltage(clamped);
        }
    }

    private void updateTargetAngleDeg(double targetDeg) {
        // Set Target
        targetAngle = targetDeg;
        controller.setGoal(targetAngle);

        // Simulation
        if (RobotBase.isSimulation() && DriveSystem.getSimulation() != null) {
            DriveSystem.getSimulation().setArmTargetAngleDeg(targetDeg);
        }
    }

    @Override
    public void periodic() {


        double rawAngleDeg = (RobotBase.isSimulation() && DriveSystem.getSimulation() != null)
            ? DriveSystem.getSimulation().getArmAngleDeg()
            : encoder.getPosition();
        double currentAngleDeg = RobotBase.isSimulation() ? rawAngleDeg : (rawAngleDeg - ArmConstants.ZERO_OFFSET_DEG);

        double pidEffort = controller.calculate(currentAngleDeg);
        TrapezoidProfile.State sp = controller.getSetpoint();
        double spPosRad = Math.toRadians(sp.position);
        double spVelRadPerS = Math.toRadians(sp.velocity);
        double ffVolts = ff.calculate(spPosRad, spVelRadPerS);

        applyMotorOutput(pidEffort + ffVolts);
    }

    @Override
    public void simulationPeriodic() {
        if (DriveSystem.getSimulation() != null) {
            double simAngleDeg = DriveSystem.getSimulation().getArmAngleDeg();
            encoder.setPosition(simAngleDeg);
        }
    }
}
