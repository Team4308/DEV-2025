package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

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
        encoder = armMotor.getEncoder();

       // armMotor.setIdleMode(IdleMode.kBrake);
        encoder.setPosition(0.0);
    }

    public void moveToScoringPosition() {
        targetAngle = ArmConstants.SCORING_ANGLE;
    }

    public void moveToRestingPosition() {
        targetAngle = ArmConstants.RESTING_ANGLE;
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
