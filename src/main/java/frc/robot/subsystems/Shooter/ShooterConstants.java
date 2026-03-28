package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

import static frc.robot.util.OCUnits.PoundSquareInches;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableUnits.TunableAngle;
import frc.robot.util.TunableUnits.TunableAngularVelocity;
import frc.robot.util.TunableUnits.TunableTime;

public final class ShooterConstants {
    
    public static final int kHoodMotorID = 41;
    public static final int kLeftMotorID = 42;
    public static final int kRightMotorID = 43;

    public static final DCMotor kFlywheelMotor = DCMotor.getKrakenX60(2);

    public static final double kFlywheelGearRatio = 20.0 / 22.0;
    public static final double kHoodGearRatio = (60.0 / 18.0) * (32.0 / 20.0) * (24.0 / 20.0) * (208.0 / 18.0);
    public static final double kBackRollerRatio = 22.0 / 36.0; // back roller upduction

    public static final MomentOfInertia kHoodMomentOfInertia = PoundSquareInches.of(168.737616);
    public static final MomentOfInertia kFlywheelMomentOfInertia = PoundSquareInches.of(8);

    public static final Distance kWheelDiameter = Inches.of(3);
    public static final Distance kBackRollerDiameter = Inches.of(1);
    public static final Distance kHoodPivotHeight = Inches.of(18.096682);
    public static final Distance kHoodLength = Inches.of(8.187500);

    // Transform from robot center to fuel exit
    public static final Transform3d kRobotToFuelExitTrf3d = new Transform3d(
        Inches.of(-6.5),
        Inches.of(5.5),
        Inches.of(19),
        new Rotation3d(0, 0, Math.PI)); // face backwards
    public static final Transform2d kRobotToFuelExitTrf2d = new Transform2d(
        kRobotToFuelExitTrf3d.getTranslation().toTranslation2d(),
        kRobotToFuelExitTrf3d.getRotation().toRotation2d()); // face backwards

    public static final AngularVelocity kFlywheelIdleVelocity = RPM.of(500);

//     public static final TunableAngularVelocity flywheelIdleVelocity = new TunableAngularVelocity("4) Shooter/Flywheel/Idle RPM", kFlywheelIdleVelocity);

    public static final Angle kHoodMinAngle = Degrees.of(21);
    public static final Angle kHoodMaxAngle = Degrees.of(45);

//     public static final TunableAngle hoodMinAngle = new TunableAngle("4) Shooter/Hood/Min Angle", kHoodMinAngle);
//     public static final TunableAngle hoodMaxAngle = new TunableAngle("4) Shooter/Hood/Max Angle", kHoodMaxAngle);

    public static final Time kFlywheelDebounceTime = Seconds.of(0.2);
    public static final Time kHoodDebounceTime = Seconds.of(0.2);

    public static final TunableTime flywheelDebounceTime = new TunableTime("4) Shooter/Flywheel/Debounce Time",
            kFlywheelDebounceTime);
    public static final TunableTime hoodDebounceTime = new TunableTime("4) Shooter/Hood/Debounce Time",
            kHoodDebounceTime);

    public static final AngularVelocity kVelocityTolerance = RPM.of(300); // TODO: tune tolerance
    public static final Angle kAngleTolerance = Degrees.of(1.5); // TODO: tune tolerance
    
    public static final TunableAngularVelocity velocityTolerance = new TunableAngularVelocity("4) Shooter/Flywheel/RPM Tolerance", kVelocityTolerance);
    public static final TunableAngle angleTolerance = new TunableAngle("4) Shooter/Hood/Degrees Tolerance", kAngleTolerance);

    public static final Time kSOTMLatency = Seconds.of(0.1);
    public static final TunableTime SOTMLatency = new TunableTime("4) Latency", kSOTMLatency);

    public static final TalonFXConfiguration kFlywheelConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kFlywheelConfig.Feedback;
        feedback.SensorToMechanismRatio = kFlywheelGearRatio;

        MotorOutputConfigs output = kFlywheelConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs current = kFlywheelConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 80;

        Slot0Configs control = kFlywheelConfig.Slot0;
        control.kP = 0.1;
        control.kI = 0;
        control.kD = 0;

        control.kS = 0;
        control.kV = 0.12;//2980 RPM
        control.kA = 0;
    }

    public static final TunableNumber flywheelkP = new TunableNumber("4) Shooter/Flywheel/PID/P",
            kFlywheelConfig.Slot0.kP);
    public static final TunableNumber flywheelkI = new TunableNumber("4) Shooter/Flywheel/PID/I",
            kFlywheelConfig.Slot0.kI);
    public static final TunableNumber flywheelkD = new TunableNumber("4) Shooter/Flywheel/PID/D",
            kFlywheelConfig.Slot0.kD);

    public static final TunableNumber flywheelkS = new TunableNumber("4) Shooter/Flywheel/Feed Forward/S",
            kFlywheelConfig.Slot0.kS);
    public static final TunableNumber flywheelkV = new TunableNumber("4) Shooter/Flywheel/Feed Forward/V",
            kFlywheelConfig.Slot0.kV);
    public static final TunableNumber flywheelkA = new TunableNumber("4) Shooter/Flywheel/Feed Forward/A",
            kFlywheelConfig.Slot0.kA);

    
    
    public static final TalonFXConfiguration kHoodConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kHoodConfig.Feedback;
        feedback.SensorToMechanismRatio = kHoodGearRatio;

        MotorOutputConfigs output = kHoodConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kHoodConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40;

        Slot0Configs control = kHoodConfig.Slot0;
        control.kP = 250; 
        control.kI = 0;
        control.kD = 0;

        control.kG = 0.2;
        control.kS = 0;
        control.kV = 0;
        control.kA = 0;

        MotionMagicConfigs mm = kHoodConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = Rotations.of(3).in(Rotations); // inches per second
        mm.MotionMagicAcceleration = Rotations.of(5).in(Rotations);
    }

    public static final TunableNumber hoodkP = new TunableNumber("4) Shooter/Hood/PID/P", kHoodConfig.Slot0.kP);
    public static final TunableNumber hoodkI = new TunableNumber("4) Shooter/Hood/PID/I", kHoodConfig.Slot0.kI);
    public static final TunableNumber hoodkD = new TunableNumber("4) Shooter/Hood/PID/D", kHoodConfig.Slot0.kD);

    public static final TunableNumber hoodkG = new TunableNumber("4) Shooter/Hood/Feed Forward/G", kHoodConfig.Slot0.kG);
    public static final TunableNumber hoodkS = new TunableNumber("4) Shooter/Hood/Feed Forward/S", kHoodConfig.Slot0.kS);
    public static final TunableNumber hoodkV = new TunableNumber("4) Shooter/Hood/Feed Forward/V", kHoodConfig.Slot0.kV);
    public static final TunableNumber hoodkA = new TunableNumber("4) Shooter/Hood/Feed Forward/A", kHoodConfig.Slot0.kA);

    public static final TunableNumber hoodCruiseVelocity = new TunableNumber("4) Shooter/Hood/MotionMagic/Cruise Velocity",
            kHoodConfig.MotionMagic.MotionMagicCruiseVelocity);
    public static final TunableNumber hoodAcceleration = new TunableNumber("4) Shooter/Hood/MotionMagic/Acceleration",
            kHoodConfig.MotionMagic.MotionMagicAcceleration);

}
