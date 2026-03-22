package frc.robot.subsystems.Indexer;

import static frc.robot.util.OCUnits.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.TunableNumber;

public class IndexerConstants {
    public static final int kSpindexerID = 31;
    public static final int kFeederID = 32;

    public static DCMotor kFeederMotor = DCMotor.getKrakenX60(1);
    public static DCMotor kSpindexerMotor = DCMotor.getKrakenX60(1);

    public static final double kSpindexerGearRatio = 3.0;
    public static final double kFeederGearRatio = 32.0 / 12.0;
    
    public static final double kSpindexerVoltage = 6;
    public static final double kFeederVoltage = 8;

    public static final TunableNumber spindexerVoltage = new TunableNumber("3) Indexer/Spindexer/Voltage", kSpindexerVoltage);
    public static final TunableNumber feederVoltage = new TunableNumber("3) Indexer/Feeder/Voltage", kFeederVoltage);
    
    public static final double kSpindexerReverseVoltage = -3;
    public static final double kFeederReverseVoltage = -4;

    public static final TunableNumber spindexerReverseVoltage = new TunableNumber("3) Indexer/Spindexer/Reverse Voltage", kSpindexerReverseVoltage);
    public static final TunableNumber feederReverseVoltage = new TunableNumber("3) Indexer/Feeder/Reverse Voltage", kFeederReverseVoltage);

//     public static final double kFeedSlowVoltage = 2;// TODO: Tune
//     public static final double kSpindexSlowVoltage = 2;// TODO: Tune

//     public static final TunableNumber feedSlowVoltage = new TunableNumber("Indexer/Feeder/Feed Slow Voltage",
//             kFeedSlowVoltage);
//     public static final TunableNumber spindexSlowVoltage = new TunableNumber("Indexer/Spindexer/Feed Slow Voltage",
//             kSpindexSlowVoltage);

    public static final MomentOfInertia kSpindexerMomentOfInertia = PoundSquareInches.of(4.9077);
    public static final MomentOfInertia kFeederMomentOfInertia = PoundSquareInches.of(0.654939);

    public static final TalonFXConfiguration kSpindexerConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kSpindexerConfig.Feedback;
        feedback.SensorToMechanismRatio = kSpindexerGearRatio;

        MotorOutputConfigs output = kSpindexerConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.Clockwise_Positive; 

        CurrentLimitsConfigs current = kSpindexerConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 60;
    }

    public static final TalonFXConfiguration kFeederConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kFeederConfig.Feedback;
        feedback.SensorToMechanismRatio = kFeederGearRatio;

        MotorOutputConfigs output = kFeederConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kFeederConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 60;
    }
}