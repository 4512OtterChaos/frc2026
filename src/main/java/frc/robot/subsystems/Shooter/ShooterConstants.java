package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.OCUnits.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.TunableNumber;

public final class ShooterConstants {
    
    public static int kHoodMotorID = 41;
    public static int kLeftMotorID = 42;
    public static int kRightMotorID = 43;

    public static double kFlywheelGearRatio = 1; // TODO: Get Later
    public static double kHoodGearRatio = 10; // TODO: Get Later

    public static double gravity = 9.8; //TODO: maybe tune later

    public static final AngularVelocity flywheelIdleVelocity = RPM.of(500);// TODO: Tune

    public static final TunableNumber flywheelIdleRPM = new TunableNumber("Shooter/Flywheel/Idle RPM", flywheelIdleVelocity.in(RPM));

    public static final Angle kHoodMinAngle = Degrees.of(0);
    public static final Angle kHoodMaxAngle = Degrees.of(30);// TODO: Tune

    public static final TunableNumber hoodMinAngle = new TunableNumber("Shooter/Hood/Min Angle", kHoodMinAngle.in(Degrees)); 
    public static final TunableNumber hoodMaxAngle = new TunableNumber("Shooter/Hood/Max Angle", kHoodMaxAngle.in(Degrees));

    public static final double kFlywheelDebounceTime = 0.25;
    public static final double kHoodDebounceTime = 0.25;

    public static final TunableNumber flywheelDebounceTime = new TunableNumber("Shooter/Flywheel/Debounce Time", kFlywheelDebounceTime);
    public static final TunableNumber hoodDebounceTime = new TunableNumber("Shooter/Hood/Debounce Time", kHoodDebounceTime);

    public static final AngularVelocity kVelocityTolerance = RPM.of(100); // TODO: tune tolerance
    public static final Angle kAngleTolerance = Degrees.of(1.5); // TODO: tune tolerance

    public static final TunableNumber RPMTolerance = new TunableNumber("Shooter/Flywheel/RPM Tolerance", kVelocityTolerance.in(RPM));
    public static final TunableNumber degreesTolerance = new TunableNumber("Shooter/Hood/Degrees Tolerance", kAngleTolerance.in(Degrees));

    public static final MomentOfInertia kMomentOfInertia = PoundSquareInches.of(80);//TODO: tune
    public static final Distance kArmLength = Inches.of(15);//TODO: tune

    public static final TalonFXConfiguration kFlywheelConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kFlywheelConfig.Feedback;
        feedback.SensorToMechanismRatio = kFlywheelGearRatio;
        
        MotorOutputConfigs output = kFlywheelConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: find direction later

        CurrentLimitsConfigs current = kFlywheelConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kFlywheelConfig.Slot0;// TODO: Tune PID
        control.kP = 3; 
        control.kI = 0;
        control.kD = 0;
        
        control.kS = 0.1;
        control.kV = 0.01;
        control.kA = 0;
    }

    public static final TunableNumber flywheelkP = new TunableNumber("Intake/Flywheel/PID/P", kFlywheelConfig.Slot0.kP);
    public static final TunableNumber flywheelkI = new TunableNumber("Intake/Flywheel/PID/I", kFlywheelConfig.Slot0.kI);
    public static final TunableNumber flywheelkD = new TunableNumber("Intake/Flywheel/PID/D", kFlywheelConfig.Slot0.kD);

    public static final TunableNumber flywheelkS = new TunableNumber("Intake/Flywheel/Feed Forward/S", kFlywheelConfig.Slot0.kS);
    public static final TunableNumber flywheelkV = new TunableNumber("Intake/Flywheel/Feed Forward/V", kFlywheelConfig.Slot0.kV);
    public static final TunableNumber flywheelkA = new TunableNumber("Intake/Flywheel/Feed Forward/A", kFlywheelConfig.Slot0.kA);

    
    public static final TalonFXConfiguration kHoodConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kHoodConfig.Feedback;
        feedback.SensorToMechanismRatio = kHoodGearRatio;
        
        MotorOutputConfigs output = kHoodConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: find direction later

        CurrentLimitsConfigs current = kHoodConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kHoodConfig.Slot0;// TODO: Tune PID
        control.kP = 9; 
        control.kI = 0;
        control.kD = 0;
        
        control.kS = 0.1;
        control.kV = 0;
        control.kA = 0;
    }
    
    public static final TunableNumber hoodkP = new TunableNumber("Intake/Hood/PID/P", kHoodConfig.Slot0.kP);
    public static final TunableNumber hoodkI = new TunableNumber("Intake/Hood/PID/I", kHoodConfig.Slot0.kI);
    public static final TunableNumber hoodkD = new TunableNumber("Intake/Climber/PID/D", kHoodConfig.Slot0.kD);

    public static final TunableNumber hoodkS = new TunableNumber("Intake/Hood/Feed Forward/S", kHoodConfig.Slot0.kS);
    public static final TunableNumber hoodkV = new TunableNumber("Intake/Hood/Feed Forward/V", kHoodConfig.Slot0.kV);
    public static final TunableNumber hoodkA = new TunableNumber("Intake/Hood/Feed Forward/A", kHoodConfig.Slot0.kA);

}
