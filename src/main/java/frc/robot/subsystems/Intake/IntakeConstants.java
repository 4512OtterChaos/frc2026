package frc.robot.subsystems.Intake;

import static frc.robot.util.OCUnits.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
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
    public static final Angle kFourBarAngleTolerance = Degrees.of(10);
    public static final TunableAngle fourBarAngleTolerance = new TunableAngle("2) Intake/Four Bar/Angle Tolerance", kFourBarAngleTolerance);
    

    // public static final TunableAngle fourBarMinAngle = new TunableAngle("2) Intake/Four Bar/Min Degrees", kFourBarMinAngle);
    // public static final TunableAngle fourBarMaxAngle = new TunableAngle("2) Intake/Four Bar/Max Degrees", kFourBarMaxAngle);

    public static final double kIntakeVoltageIn = 9; 
    public static final double kIntakeVoltageOut = -6;

    // public static final TunableNumber intakeVoltageIn = new TunableNumber("2) Intake/Roller/Voltage In", kIntakeVoltageIn);
    // public static final TunableNumber intakeVoltageOut = new TunableNumber("2) Intake/Roller/Voltage Out", kIntakeVoltageOut);

    public static final Current kRetractCurrent1 = Amps.of(25);
    public static final Current kRetractCurrent2 = Amps.of(10);
    public static final Current kExtendCurrent1 = Amps.of(-25);
    public static final Current kExtendCurrent2 = Amps.of(-7);

    public static final TunableCurrent retractCurrent1 = new TunableCurrent("2) Intake/Four Bar/Retract Amps 1", kRetractCurrent1);
    public static final TunableCurrent retractCurrent2 = new TunableCurrent("2) Intake/Four Bar/Retract Amps 2", kRetractCurrent2);
    public static final TunableCurrent extendCurrent1 = new TunableCurrent("2) Intake/Four Bar/Extend Amps 1", kExtendCurrent1);
    public static final TunableCurrent extendCurrent2 = new TunableCurrent("2) Intake/Four Bar/Extend Amps 2", kExtendCurrent2);

    // public static final Current kSmallAmpsIn = Amps.of(5); //TODO: Tune
    // public static final Current kSmallCurrentOut = Amps.of(-5); //TODO: Tune

    // public static final TunableCurrent smallCurrentIn = new TunableCurrent("Intake/Four Bar/Small Current In", kSmallCurrentIn);
    // public static final TunableCurrent smallCurrentOut = new TunableCurrent("Intake/Four Bar/Small Current Out", kSmallCurrentOut);

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
        current.StatorCurrentLimit = 70;
        current.SupplyCurrentLimitEnable = true;
        current.SupplyCurrentLimit = 55;
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
    }
}
