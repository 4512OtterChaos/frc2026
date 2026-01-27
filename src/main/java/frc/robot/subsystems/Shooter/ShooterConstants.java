package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShooterConstants {
    
    public static int kHoodMotorID = 41;
    public static int kLeftMotorID = 42;
    public static int kRightMotorID = 43;

    public static double kFlywheelGearRatio = 1; // TODO: Get Later
    public static double kHoodGearRatio = 1; // TODO: Get Later

    public static Angle kHoodMinAngle = Degrees.of(0);
    public static Angle kHoodMaxAngle = Degrees.of(30);

    public static final double kDebounceTime = 0.25;
    
    public static final AngularVelocity kVelocityTolerance = RPM.of(100); // TODO: tune tolerance
    public static final Angle kAngleTolerance = Degrees.of(1.5); // TODO: tune tolerance

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

        Slot0Configs control = kFlywheelConfig.Slot0;
        control.kP = 0.1; 
        control.kI = 0;
        control.kD = 0.1;
        
        control.kS = 0.1;
        control.kV = 0.01;
        control.kA = 0;
    }

    
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

        Slot0Configs control = kHoodConfig.Slot0;
        control.kP = 0.1; 
        control.kI = 0;
        control.kD = 0.1;
        
        control.kS = 0.1;
        control.kV = 0.01;
        control.kA = 0;
    }

}
