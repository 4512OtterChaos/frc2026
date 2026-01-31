package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;

public class ClimberConstants {
    public static final int kMotorID = 51; 

    public static final Distance kMaxHeight = Inches.of(30); //TODO: tune
    public static final Distance kMinHeight = Inches.of(23); //TODO: tune
    public static final Distance kHeightTolerance = Inches.of(0.25); //TODO: tune

    public static final double kDebounceTime = 0.25;

    public static final int kGearRatio = 1; //TODO: tune

    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = kGearRatio;
        
        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: find direction later

        CurrentLimitsConfigs current = kConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kConfig.Slot0;// TODO: Tune PID
        control.kP = 0.1; 
        control.kI = 0;
        control.kD = 0;
        
        control.kS = 0.1;
        control.kV = 0.01;
        control.kA = 0;
    }

}
