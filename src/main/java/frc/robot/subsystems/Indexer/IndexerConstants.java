package frc.robot.subsystems.Indexer;

import static frc.robot.util.OCUnits.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.TunableNumber;


public class IndexerConstants {
    public static final int kSpindexerID = 31;
    public static final int kFeederID = 32;

    public static final int kSpindexerGearRatio = 3;
    public static final int kFeederGearRatio = 32/12;

    public static final double kSpindexerVoltage = 4;// TODO: Tune
    public static final double kFeederVoltage = 4;// TODO: Tune

    public static final TunableNumber spindexerVoltage = new TunableNumber("Indexer/Spindexer/Volatge", kSpindexerVoltage); 
    public static final TunableNumber feederVoltage = new TunableNumber("Indexer/Feeder/Voltage", kFeederVoltage); 

    public static final double kFeedSlowVoltage = 2;// TODO: Tune
    public static final double kSpindexSlowVoltage = 2;// TODO: Tune

    public static final TunableNumber feedSlowVoltage = new TunableNumber("Indexer/Feeder/Feed Slow Voltage", kFeedSlowVoltage); 
    public static final TunableNumber spindexSlowVoltage = new TunableNumber("Indexer/Spindexer/Feed Slow Voltage", kSpindexSlowVoltage); 

    public static final MomentOfInertia kSpindexerMomentOfInertia = PoundSquareInches.of(4.9077);
    public static final MomentOfInertia kFeederMomentOfInertia = PoundSquareInches.of(0.654939);
    

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
        control.kD = 0;
        
        control.kS = 0.1;
        control.kV = 0.01;
        control.kA = 0;
    }

    public static final TunableNumber spindexerkP = new TunableNumber("Indexer/Spindexer/PID/P", kSpindexerConfig.Slot0.kP);
    public static final TunableNumber spindexerkI = new TunableNumber("Indexer/Spindexer/PID/I", kSpindexerConfig.Slot0.kI);
    public static final TunableNumber spindexerkD = new TunableNumber("Indexer/Spindexer/PID/D", kSpindexerConfig.Slot0.kD);

    public static final TunableNumber spindexerkS = new TunableNumber("Indexer/Spindexer/Feed Forward/S", kSpindexerConfig.Slot0.kS);
    public static final TunableNumber spindexerkV = new TunableNumber("Indexer/Spindexer/Feed Forward/V", kSpindexerConfig.Slot0.kV); 
    public static final TunableNumber spindexerkA = new TunableNumber("Indexer/Spindexer/Feed Forward/A", kSpindexerConfig.Slot0.kA);
    

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

    public static final TunableNumber feederkP = new TunableNumber("Indexer/Feeder/PID/P", kFeederConfig.Slot0.kP);
    public static final TunableNumber feederkI = new TunableNumber("Indexer/Feeder/PID/I", kFeederConfig.Slot0.kI);
    public static final TunableNumber feederkD = new TunableNumber("Indexer/Feeder/PID/D", kFeederConfig.Slot0.kD);

    public static final TunableNumber feederkS = new TunableNumber("Indexer/Feeder/Feed Forward/S", kFeederConfig.Slot0.kS);
    public static final TunableNumber feederkV = new TunableNumber("Indexer/Feeder/Feed Forward/V", kFeederConfig.Slot0.kV); 
    public static final TunableNumber feederkA = new TunableNumber("Indexer/Feeder/Feed Forward/A", kFeederConfig.Slot0.kA);



}  