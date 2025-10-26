package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// Removed unused RelativeEncoder import
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final SparkAbsoluteEncoder encoder;
    private final ProfiledPIDController controller = new ProfiledPIDController(
        ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.MAX_VEL_DEG_PER_S, ArmConstants.MAX_ACC_DEG_PER_S2)
    );
    private final ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    private double targetAngle = ArmConstants.RESTING_ANGLE;

    private final boolean simActive = RobotBase.isSimulation();
    private Simulation sim = null;
    private double lastVolts = 0.0;
    private double manualPercent = 0.0;

    public ArmSubsystem() {
        armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);

        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.absoluteEncoder.zeroOffset(ArmConstants.ZERO_OFFSET_DEG);
        armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        encoder = armMotor.getAbsoluteEncoder();

        double initAngleDeg = encoder.getPosition();
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

    public void setManualPercent(double percent) {
        manualPercent = Math.max(-1.0, Math.min(1.0, percent));
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
        SmartDashboard.putNumber("Angle Arm", encoder.getPosition());
        double currentAngleDeg = rawAngleDeg;

        double pct = manualPercent;
        if (Math.abs(pct) > 0.05) { 
            applyMotorOutput(pct * 12.0);
            controller.reset(currentAngleDeg);
            controller.setGoal(currentAngleDeg);
            if (simActive && sim != null) {
                sim.setArmTargetAngleDeg(currentAngleDeg);
            }
            return;
        }

        double pidEffort = controller.calculate(currentAngleDeg);
        TrapezoidProfile.State sp = controller.getSetpoint();
        double ffVolts = ff.calculate(Math.toRadians(sp.position), Math.toRadians(sp.velocity));
        applyMotorOutput(pidEffort + ffVolts);

        if (simActive && sim != null) {
            sim.setArmTargetAngleDeg(controller.getGoal().position);
        }
    }
}
