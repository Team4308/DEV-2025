package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;

    private final PIDController pid = new PIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD
    );

    private final ArmFeedforward ff = new ArmFeedforward(
        ArmConstants.kS,
        ArmConstants.kG,
        ArmConstants.kV
    );

    private double targetAngle = ArmConstants.RESTING_ANGLE;

    public ArmSubsystem() {
        armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


        encoder = armMotor.getEncoder();        
        encoder.setPosition(0.0);
    }

    public void moveToScoringPosition() {
        targetAngle = ArmConstants.SCORING_ANGLE;
    }

    public void moveToRestingPosition() {
        targetAngle = ArmConstants.RESTING_ANGLE;
    }
    
    public void moveToFeederPosition() {
        targetAngle = ArmConstants.FEEDER_ANGLE;
    }

    public void moveToGroundIntakePosition() {
        targetAngle = ArmConstants.GROUND_ANGLE;
    }

    public int getCurrentAngle() {
        return (int) encoder.getPosition();
    }
    public int getTargetAngle() {
        return (int) targetAngle;
    }

    public double getVoltage() {
        return armMotor.getAppliedOutput();
    }


    @Override
    public void periodic() {
        double currentPos = encoder.getPosition();
        double pidOutput = pid.calculate(currentPos, targetAngle);
        double ffOutput = ff.calculate(targetAngle, 0);
        double totalOutput = pidOutput + ffOutput;

        armMotor.setVoltage(totalOutput);
    }
}
