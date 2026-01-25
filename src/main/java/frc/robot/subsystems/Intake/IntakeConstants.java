package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static final int kIntakeMotorID = 21;
    public static final int kFourBarMotorID = 22;

    public static final double kIntakeVoltageIn = 2.5; //TODO: Tune
    public static final double kIntakeVoltageOut = -2.5; //TODO: Tune
    public static final double kFourBarVoltageIn = 2.5; //TODO: Tune
    public static final double kFourBarVoltageOut = -2.5; //TODO: Tune

    public static final double kIntakeGearRatio = 1; //TODO: Use Real
    public static final double kFourBarGearRatio = 1; //TODO: Use Real

    public static final TalonFXConfiguration kIntakeConfig = new TalonFXConfiguration();
    static { 
        FeedbackConfigs feedback = kIntakeConfig.Feedback;
        feedback.SensorToMechanismRatio = kIntakeGearRatio;

        MotorOutputConfigs output = kIntakeConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kIntakeConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 
    }

    public static final TalonFXConfiguration kFourBarConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kFourBarConfig.Feedback;
        feedback.SensorToMechanismRatio = kFourBarGearRatio;

        MotorOutputConfigs output = kFourBarConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kFourBarConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 
    }
}
