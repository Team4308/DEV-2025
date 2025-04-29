package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase  {
    private final TalonSRX intakeMotor = new TalonSRX(Intake.Intake_Motor_id);



    public IntakeSubsystem() {
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.slot0.kP = 0.12;
        config.slot0.kI = 0.11;
        config.slot0.kD = 0.5;
        config.slot0.kF = 0.001;
        intakeMotor.configAllSettings(config);
    }

    public void setInverted(boolean inverted) {
        intakeMotor.setInverted(inverted);
    }

    public void setMotorSpeed(double percent) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput,percent);
    }
    public void stop() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }


    public void intake() {
        setMotorSpeed(25);
    }

    public void outtake() {
        setMotorSpeed(-25);
    }

}
