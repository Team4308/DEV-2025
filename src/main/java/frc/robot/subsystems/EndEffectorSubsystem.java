package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class EndEffectorSubsystem extends SubsystemBase  {
    private final TalonFX intakeMotor = new TalonFX(Intake.Intake_Motor_id);
    public final DigitalInput intakeSensor = new DigitalInput(Intake.Intake_Sensor_id);

    public EndEffectorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Intake.kP;
        config.Slot0.kI = Intake.kI;
        config.Slot0.kD = Intake.kD;

        
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.getConfigurator().apply(config);
    }

    public void setInverted(boolean inverted) {
        intakeMotor.setInverted(inverted);
    }

    public void setMotorSpeed(double percent) {

        intakeMotor.set(percent);
    }
    public void stop() {
        intakeMotor.set(0);
    }


    public void intake() {
        setMotorSpeed(0.25);
    }

    public void outtake() {
        setMotorSpeed(-0.25);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Intook", intakeSensor.get());
        SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.get());
        if (intakeSensor.get()) {
            stop();
        }
    }
}
