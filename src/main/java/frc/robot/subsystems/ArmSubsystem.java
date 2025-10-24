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
        ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.MAX_VEL_DEG_PER_S, ArmConstants.MAX_ACC_DEG_PER_S2)
    );
    private final ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    private double targetAngle = ArmConstants.RESTING_ANGLE;

    private final boolean simActive = RobotBase.isSimulation();
    private Simulation sim = null;
    private double lastVolts = 0.0;

    public ArmSubsystem() {
        armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.encoder.positionConversionFactor(ArmConstants.DEG_PER_ENCODER_UNIT);
        config.encoder.velocityConversionFactor(ArmConstants.DEG_PER_ENCODER_UNIT / 60.0);
        armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        encoder = armMotor.getEncoder();

        double initAngleDeg = encoder.getPosition() - ArmConstants.ZERO_OFFSET_DEG;
        targetAngle = initAngleDeg;
        controller.reset(initAngleDeg);
        controller.setGoal(targetAngle);
        controller.setTolerance(1.0);

        if (simActive) {
            sim = DriveSystem.getSimulation();
        }
    }

    public void moveToScoringPosition() { updateTargetAngleDeg(ArmConstants.SCORING_ANGLE); }
    public void moveToRestingPosition() { updateTargetAngleDeg(ArmConstants.RESTING_ANGLE); }
    public void moveToFeederPosition()  { updateTargetAngleDeg(ArmConstants.FEEDER_ANGLE); }
    public void moveToGroundIntakePosition() { updateTargetAngleDeg(ArmConstants.GROUND_ANGLE); }

    public int getCurrentAngle() {
        if (simActive && sim != null) return (int) sim.getArmAngleDeg();
        return (int) encoder.getPosition();
    }
    public int getTargetAngle()  { return (int) targetAngle; }
    public double getVoltage()   { return simActive ? lastVolts : armMotor.getAppliedOutput(); }

    private void applyMotorOutput(double volts) {
        double clamped = Math.max(-12.0, Math.min(12.0, volts));
        lastVolts = clamped;
        if (simActive && sim != null) {
            sim.setArmVoltage(clamped);
        } else {
            armMotor.setVoltage(clamped);
        }
    }

    private void updateTargetAngleDeg(double targetDeg) {
        targetAngle = targetDeg;
        controller.setGoal(targetAngle);
        if (simActive && sim != null) {
            sim.setArmTargetAngleDeg(targetAngle);
        }
    }

    @Override
    public void periodic() {
        if (simActive && sim == null) {
            sim = DriveSystem.getSimulation();
            if (sim != null) {
                sim.setArmTargetAngleDeg(targetAngle);
                sim.setArmVoltage(lastVolts);
            }
        }

        double rawAngleDeg = simActive && sim != null ? sim.getArmAngleDeg() : encoder.getPosition();
        double currentAngleDeg = rawAngleDeg - ArmConstants.ZERO_OFFSET_DEG;
        double pidEffort = controller.calculate(currentAngleDeg);
        TrapezoidProfile.State sp = controller.getSetpoint();
        double ffVolts = ff.calculate(Math.toRadians(sp.position), Math.toRadians(sp.velocity));
        applyMotorOutput(pidEffort + ffVolts);

        if (simActive && sim != null) {
            sim.setArmTargetAngleDeg(controller.getGoal().position);
        }
    }
}
