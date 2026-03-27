package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.OCUnits.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableUnits.TunableAngularVelocity;

public class IndexerConstants {
    public static final int kSpindexerID = 31;
    public static final int kFeederID = 32;

    public static DCMotor kFeederMotor = DCMotor.getKrakenX60(1);
    public static DCMotor kSpindexerMotor = DCMotor.getKrakenX60(1);

    public static final double kSpindexerGearRatio = 3.0;
    public static final double kFeederGearRatio = 32.0 / 12.0;
    
    public static final double kSpindexerVoltage = 6;
    public static final double kFeederVoltage = 9;

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

    public static final AngularVelocity kFeederVelocity = RPM.of(1600);
    public static final TunableAngularVelocity feederVelocity = new TunableAngularVelocity("3) Indexer/Feeder/Velocity", kFeederVelocity);

    public static final AngularVelocity kFeederReverseVelocity = RPM.of(-750);
    public static final TunableAngularVelocity feederReverseVelocity = new TunableAngularVelocity("3) Indexer/Feeder/Reverse Velocity", kFeederReverseVelocity);


    public static final AngularVelocity kFeederVelocityTolerance = RPM.of(100);
    public static final TunableAngularVelocity feederVelocityTolerance = new TunableAngularVelocity("3) Indexer/Feeder/Velocity", kFeederVelocityTolerance);

    public static final double kFeederDebounceTime = 0.2;
    public static final TunableNumber feederDebounceTime = new TunableNumber("3) Indexer/Feeder/Velocity Debounce", kFeederDebounceTime);
    
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
        current.StatorCurrentLimit = 70;
        current.SupplyCurrentLimitEnable = true;
        current.SupplyCurrentLimit = 60;

        Slot0Configs control = kFeederConfig.Slot0;
        control.kP = 0;
        control.kI = 0;
        control.kD = 0;

        control.kS = 0;
        control.kV = 0.34;
        control.kA = 0;
    }

    public static final TunableNumber feederkP = new TunableNumber("3) Indexer/Feeder/PID/P",
            kFeederConfig.Slot0.kP);
    public static final TunableNumber feederkI = new TunableNumber("3) Indexer/Feeder/PID/I",
            kFeederConfig.Slot0.kI);
    public static final TunableNumber feederkD = new TunableNumber("3) Indexer/Feeder/PID/D",
            kFeederConfig.Slot0.kD);

    public static final TunableNumber feederkS = new TunableNumber("3) Indexer/Feeder/Feed Forward/S",
            kFeederConfig.Slot0.kS);
    public static final TunableNumber feederkV = new TunableNumber("3) Indexer/Feeder/Feed Forward/V",
            kFeederConfig.Slot0.kV);
    public static final TunableNumber feederkA = new TunableNumber("3) Indexer/Feeder/Feed Forward/A",
            kFeederConfig.Slot0.kA);
}