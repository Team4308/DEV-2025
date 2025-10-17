package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.BotState;

public class DeepClimbSubsystem extends SubsystemBase {
    
    private TalonSRX motor = new TalonSRX(Constants.Deepclimb.MOTORID);
    

    public DeepClimbSubsystem() {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.configPeakOutputForward(1.0);
        motor.configPeakOutputReverse(-1.0);
        motor.configContinuousCurrentLimit(20, 0);
        motor.enableCurrentLimit(true);
        motor.setNeutralMode(NeutralMode.Brake);
        
    }

    public void startClimb() {
        RobotContainer.currentState = BotState.CLIMBING;
        motor.set(ControlMode.MotionMagic.PercentOutput, 55);
    }

    public void stopClimb() {
        RobotContainer.currentState = BotState.IDLE;

        motor.set(ControlMode.MotionMagic.PercentOutput, 0);
    }

}
