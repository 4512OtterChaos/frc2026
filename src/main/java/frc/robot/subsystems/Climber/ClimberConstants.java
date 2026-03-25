package frc.robot.subsystems.Climber;

import static frc.robot.util.OCUnits.PoundSquareInches;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableUnits.TunableAngle;
import frc.robot.util.TunableUnits.TunableDistance;

public class ClimberConstants {
    public static final int kMotorID = 51; 
    public static final double kGearRatio = 27;
    public static final MomentOfInertia kMomentOfInertia = PoundSquareInches.of(80);//TODO: tune

    public static final Angle kMaxAngle = Rotations.of(5); //TODO: tune
    public static final Angle kMinAngle = Rotations.of(0); //TODO: tune
    public static final Angle kAngleTolerance = Rotations.of(0.2); //TODO: tune

    public static final TunableAngle maxAngle = new TunableAngle("5) Climber/Max Angle", kMaxAngle);
    public static final TunableAngle minAngle = new TunableAngle("5) Climber/Min Angle", kMinAngle);
    public static final TunableAngle angleTolerance = new TunableAngle("5) Climber/Angle Tolerance", kAngleTolerance);


    public static final Distance kMinHeight = Inches.of(22); //TODO: tune
    public static final Distance kMaxHeight = Inches.of(30); //TODO: tune

    public static final TunableDistance maxHeight = new TunableDistance("5) Climber/Max Height", kMaxHeight);
    public static final TunableDistance minHeight = new TunableDistance("5) Climber/Min Height", kMinHeight);

    public static final double kDebounceTime = 0.25;
    public static final TunableNumber debounceTime = new TunableNumber("5) Climber/Debounce Time", kDebounceTime);

    //CLIMBER sim
    public static final double climberWeight = 1; //TODO: find weight
    public static final Distance shaftRad = Inches.of((maxHeight.in(Inches) - minHeight.in(Inches)) / maxAngle.in(Rotations) / 2 * Math.PI);

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
        control.kP = 3; 
        control.kI = 0;
        control.kD = 0;
        
        control.kS = 0;
        control.kV = 0;
        control.kA = 0;

        MotionMagicConfigs mm = kConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = Rotations.of(3).in(Rotations); // inches per second
        mm.MotionMagicAcceleration = Rotations.of(5).in(Rotations);
    }

    public static final TunableNumber kP = new TunableNumber("Climber/PID/P", kConfig.Slot0.kP);
    public static final TunableNumber kI = new TunableNumber("Climber/PID/I", kConfig.Slot0.kI);
    public static final TunableNumber kD = new TunableNumber("Climber/PID/D", kConfig.Slot0.kD);
    
    public static final TunableNumber kS = new TunableNumber("Climber/Feed Forward/S", kConfig.Slot0.kS);
    public static final TunableNumber kV = new TunableNumber("Climber/Feed Forward/V", kConfig.Slot0.kV);
    public static final TunableNumber kA = new TunableNumber("Climber/Feed Forward/A", kConfig.Slot0.kA);

    public static Angle heightToAngle(Distance height){
        return Rotations.of(height.in(Inches) * maxAngle.in(Rotations) / (maxHeight.in(Inches) - minHeight.in(Inches)));
    }

    public static Distance angleToHeight(Angle angle){
        return Inches.of(angle.in(Rotations) * (maxHeight.in(Inches) - minHeight.in(Inches)) / maxAngle.in(Rotations));
    }
}