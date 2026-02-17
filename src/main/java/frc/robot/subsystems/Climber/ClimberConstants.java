package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.OCUnits.PoundSquareInches;

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
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimberConstants {
    public static final int kMotorID = 51; 

    public static final Distance kMaxHeightRot = Meters.of(2); //TODO: tune
    public static final Distance kMinHeightRot = Meters.of(2); //TODO: tune
    public static final Distance kHeightTolerance = Meters.of(0.2); //TODO: tune

    //CLIMBER sim
    public static final double climberWeight = 6; //TODO: find weight
    public static final double wheelRad = 5;
    public static final Distance kSprocketPD = Inches.of(1.76); //TODO: find out what a spricket is


    public static final MomentOfInertia kMomentOfInertia = PoundSquareInches.of(80);//TODO: tune

    public static final TunableNumber maxHeight = new TunableNumber("Climber/Max Height", kMaxHeightRot.in(Meters));
    public static final TunableNumber minHeight = new TunableNumber("Climber/Max Height", kMinHeightRot.in(Meters));
    public static final TunableNumber heightTolerance = new TunableNumber("Climber/Height Tolerance", kHeightTolerance.in(Meters));

    public static final double kDebounceTime = 0.25;

    public static final TunableNumber debounceTime = new TunableNumber("Climber/Debounce Time", kDebounceTime);

    public static final double kGearRatio = 1; //TODO: tune

    public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kConfig.Feedback;
        feedback.SensorToMechanismRatio = kGearRatio;
        
        MotorOutputConfigs output = kConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
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

    public static Angle carriageDistToMotorAngle(Distance dist) {
        return Rotations.of(dist.in(Meters) / 2.0 / (kSprocketPD.in(Meters) * Math.PI) * kGearRatio);
    }

    public static final TunableNumber kP = new TunableNumber("Climber/PID/P", kConfig.Slot0.kP);
    public static final TunableNumber kI = new TunableNumber("Climber/PID/I", kConfig.Slot0.kI);
    public static final TunableNumber kD = new TunableNumber("Climber/PID/D", kConfig.Slot0.kD);
    
    public static final TunableNumber kS = new TunableNumber("Climber/Feed Forward/S", kConfig.Slot0.kS);
    public static final TunableNumber kV = new TunableNumber("Climber/Feed Forward/V", kConfig.Slot0.kV);
    public static final TunableNumber kA = new TunableNumber("Climber/Feed Forward/A", kConfig.Slot0.kA);


}
