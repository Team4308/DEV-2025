package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    private static final int ARM_MOTOR_ID = 10;
    private static final double SCORING_ANGLE = 30.0;
    private static final double RESTING_ANGLE = 0.0;

    public ArmSubsystem() {
        armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
        pid = armMotor.getPIDController();

        armMotor.setIdleMode(IdleMode.kBrake);
        encoder.setPosition(0);

        pid.setP(0.01);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.0);

        pid.setOutputRange(-1.0, 1.0);
    }

    public void moveToScoringPosition() {
        pid.setReference(SCORING_ANGLE, CANSparkMax.ControlType.kPosition);
    }

    public void moveToRestingPosition() {
        pid.setReference(RESTING_ANGLE, CANSparkMax.ControlType.kPosition);
    }
}
