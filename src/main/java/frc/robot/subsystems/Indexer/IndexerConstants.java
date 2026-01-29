package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
    public static final int kSpindexerID = 31;
    public static final int kFeederID = 32;

    public static final int kSpindexerGearRatio = 1;// TODO: Tune
    public static final int kFeederGearRatio = 1;// TODO: Tune

    public static final double kSpindexerVoltage = 4;// TODO: Tune
    public static final double kFeederVoltage = 4;// TODO: Tune

    public static final double kFeedSlowVoltage = 2;// TODO: Tune
    public static final double kSpindexSlowVoltage = 2;// TODO: Tune


    public static final TalonFXConfiguration kSpindexerConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kSpindexerConfig.Feedback;
        feedback.SensorToMechanismRatio = kSpindexerGearRatio;
        
        MotorOutputConfigs output = kSpindexerConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: find direction later

        CurrentLimitsConfigs current = kSpindexerConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kSpindexerConfig.Slot0;// TODO: Tune PID
        control.kP = 0.1; 
        control.kI = 0;
        control.kD = 0.1;
        
        control.kS = 0.1;
        control.kV = 0.01;
        control.kA = 0;
    }
    
    public static final TalonFXConfiguration kFeederConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kFeederConfig.Feedback;
        feedback.SensorToMechanismRatio = kFeederGearRatio;
        
        MotorOutputConfigs output = kFeederConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: find direction later

        CurrentLimitsConfigs current = kFeederConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kFeederConfig.Slot0;// TODO: Tune PID
        control.kP = 0.1; 
        control.kI = 0;
        control.kD = 0;
        
        control.kS = 0.1;
        control.kV = 0.01;
        control.kA = 0;
    }

}
