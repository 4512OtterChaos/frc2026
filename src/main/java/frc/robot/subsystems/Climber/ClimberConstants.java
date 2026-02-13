package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import frc.robot.util.TunableNumber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ClimberConstants {
    public static final int kMotorID = 51; 

    public static final Angle kMaxHeightRot = Degrees.of(3000); //TODO: tune
    public static final Angle kMinHeightRot = Degrees.of(2300); //TODO: tune
    public static final Angle kHeightTolerance = Degrees.of(100); //TODO: tune

    public static final TunableNumber maxHeight = new TunableNumber("Climber/Max Height", kMaxHeightRot.in(Degrees));
    public static final TunableNumber minHeight = new TunableNumber("Climber/Max Height", kMinHeightRot.in(Degrees));
    public static final TunableNumber heightTolerance = new TunableNumber("Climber/Height Tolerance", kHeightTolerance.in(Degrees));

    public static final double kDebounceTime = 0.25;

    public static final TunableNumber debounceTime = new TunableNumber("Climber/Debounce Time", kDebounceTime);

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

    public static final TunableNumber kP = new TunableNumber("Intake/Climber/PID/P", kConfig.Slot0.kP);
    public static final TunableNumber kI = new TunableNumber("Intake/Climber/PID/I", kConfig.Slot0.kI);
    public static final TunableNumber kD = new TunableNumber("Intake/Climber/PID/D", kConfig.Slot0.kD);
    
    public static final TunableNumber kS = new TunableNumber("Intake/Climber/Feed Forward/S", kConfig.Slot0.kS);
    public static final TunableNumber kV = new TunableNumber("Intake/Climber/Feed Forward/V", kConfig.Slot0.kV);
    public static final TunableNumber kA = new TunableNumber("Intake/Climber/Feed Forward/A", kConfig.Slot0.kA);


}
