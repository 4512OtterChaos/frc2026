package frc.robot.subsystems.Intake;

import static frc.robot.util.OCUnits.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableUnits.TunableAngle;
import frc.robot.util.TunableUnits.TunableCurrent;

public class IntakeConstants {
    public static final int kIntakeMotorID = 21;
    public static final int kFourBarMotorID = 22;

    public static final double kIntakeGearRatio = 2; 
    public static final double kFourBarGearRatio = (38.0/8.0) * (38.0/20.0) * (26.0/12.0); 

    public static final Distance kFourBarArmLength = Feet.of(2);

    public static final Angle kFourBarMinAngle = Degrees.of(16.5); 
    public static final Angle kFourBarMaxAngle = Degrees.of(100.5); 
    public static final Angle kAngleTolerance = Degrees.of(2); //TODO: tune

    public static final TunableAngle fourBarMinAngle = new TunableAngle("2) Intake/Four Bar/Min Degrees", kFourBarMinAngle);
    public static final TunableAngle fourBarMaxAngle = new TunableAngle("2) Intake/Four Bar/Max Degrees", kFourBarMaxAngle);
    public static final TunableAngle angleTolerance = new TunableAngle("2) Intake/Four Bar/Degrees Tolerance", kAngleTolerance);

    public static final double kIntakeVoltageIn = 8; //TODO: Tune
    public static final double kIntakeVoltageOut = -4; //TODO: Tune

    public static final double kFourBarVoltageIn = 2; //TODO: Tune
    public static final double kFourBarVoltageOut = -2; //TODO: Tune

    public static final TunableNumber intakeVoltageIn = new TunableNumber("2) Intake/Roller/Voltage In", kIntakeVoltageIn);
    public static final TunableNumber intakeVoltageOut = new TunableNumber("2) Intake/Roller/Voltage Out", kIntakeVoltageOut);
    public static final TunableNumber fourBarVoltageIn = new TunableNumber("2) Intake/Four Bar/Voltage In", kFourBarVoltageIn);
    public static final TunableNumber fourBarVoltageOut = new TunableNumber("2) Intake/Four Bar/Voltage Out", kFourBarVoltageOut);

    public static final Current kCurrentIn = Amps.of(20); //TODO: Tune
    public static final Current kCurrentOut = Amps.of(-20);

    // public static final Current kSmallAmpsIn = Amps.of(5); //TODO: Tune
    // public static final Current kSmallAmpsOut = Amps.of(-5); //TODO: Tune

    public static final TunableCurrent currentIn = new TunableCurrent("2) Intake/Four Bar/Amps In", kCurrentIn);
    public static final TunableCurrent currentOut = new TunableCurrent("2) Intake/Four Bar/Amps Out", kCurrentOut);

    // public static final TunableNumber smallAmpsIn = new TunableNumber("Intake/Four Bar/Small Amps In", kSmallAmpsIn.in(Amps));
    // public static final TunableNumber smallAmpsOut = new TunableNumber("Intake/Four Bar/Small Amps Out", kSmallAmpsIn.in(Amps));

    public static final MomentOfInertia kFourBarMomentOfInertia = PoundSquareInches.of(10);//TODO: tune
    public static final MomentOfInertia kIntakeMomentOfInertia = PoundSquareInches.of(10);//TODO: tune

    public static final Distance kFourBar1PivotHeight = Inches.of(7.250000);
    public static final Distance kFourBar1Length = Inches.of(10.248965);

    public static final Distance kFourBar2PivotHeight = Inches.of(6.750000);
    public static final Distance kFourBar2Length = Inches.of(11.617010);

    public static final TalonFXConfiguration kIntakeConfig = new TalonFXConfiguration();
    static { 
        FeedbackConfigs feedback = kIntakeConfig.Feedback;
        feedback.SensorToMechanismRatio = kIntakeGearRatio;

        MotorOutputConfigs output = kIntakeConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kIntakeConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 55; 

        Slot0Configs control = kIntakeConfig.Slot0;// TODO: Tune PID
            control.kP = 9; 
            control.kI = 0;
            control.kD = 0;
            
            control.kS = 0;
            control.kV = 0;
            control.kA = 0;

    }

    public static final TalonFXConfiguration kFourBarConfig = new TalonFXConfiguration();
    static {
        FeedbackConfigs feedback = kFourBarConfig.Feedback;
        feedback.SensorToMechanismRatio = kFourBarGearRatio;

        MotorOutputConfigs output = kFourBarConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kFourBarConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 50; 

        Slot0Configs control = kFourBarConfig.Slot0;// TODO: Tune PID
            control.kP = 20; 
            control.kI = 0.1;
            control.kD = 0;
            
            control.kS = 0;
            control.kV = 0;
            control.kA = 0;
            
        MotionMagicConfigs mm = kFourBarConfig.MotionMagic;
        mm.MotionMagicCruiseVelocity = Rotations.of(3).in(Rotations); // inches per second
        mm.MotionMagicAcceleration = Rotations.of(5).in(Rotations);
    }
}
