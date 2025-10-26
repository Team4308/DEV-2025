package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.RobotContainer.BotState;
import frc.robot.commands.score;
import frc.robot.commands.Sequential.CoralIntakeCommand;

public class EndEffectorSubsystem extends SubsystemBase  {
    private final TalonSRX intakeMotor = new TalonSRX(Intake.Intake_Motor_id);
    public final DigitalInput intakeSensor = new DigitalInput(Intake.Intake_Sensor_id);

    public EndEffectorSubsystem() {
        TalonSRXConfiguration intakeConfig = new TalonSRXConfiguration();
        intakeConfig.slot0.kP = Intake.kP;
        intakeConfig.slot0.kI = Intake.kI;
        intakeConfig.slot0.kD = Intake.kD;
        intakeMotor.configAllSettings(intakeConfig);
        intakeMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    }

    public void setInverted(boolean inverted) {
        intakeMotor.setInverted(inverted);
    }

    public void setMotorSpeed(double percent) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, percent);
    }
    public void stop() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }


    public void intake() {
        setMotorSpeed(0.35);
    }

    public void outtake() {
        setMotorSpeed(-0.25);
    }

    private BotState previousState = null;

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Coral Intook", intakeSensor.get());
        SmartDashboard.putNumber("Intake Motor Speed", intakeMotor.getMotorOutputPercent());

        if (RobotContainer.currentState != previousState) {
            if (RobotContainer.currentState == BotState.INTAKING) {
                CommandScheduler.getInstance().schedule(new CoralIntakeCommand(this, RobotContainer.m_armSubsystem, RobotContainer.groundIntake()));
            } else if (RobotContainer.currentState == BotState.SCORING) {
                CommandScheduler.getInstance().schedule(new score(this, RobotContainer.m_armSubsystem));
            }
            previousState = RobotContainer.currentState;
        }
    }
}
