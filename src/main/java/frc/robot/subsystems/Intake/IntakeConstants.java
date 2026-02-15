package frc.robot.subsystems.Intake;
import static frc.robot.util.OCUnits.*;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.util.TunableNumber;

public class IntakeConstants {
    public static final int kIntakeMotorID = 21;
    public static final int kFourBarMotorID = 22;

    public static final double kIntakeGearRatio = 1; //TODO: Use Real
    public static final double kFourBarGearRatio = 1; //TODO: Use Real

    public static final Distance kFourBarArmLength = Feet.of(2);

    public static final Angle kFourBarMinAngle = Degrees.of(15); //TODO: tune
    public static final Angle kFourBarMaxAngle = Degrees.of(85); //TODO: tune
    public static final Angle kAngleTolerance = Degrees.of(2); //TODO: tune

    public static final TunableNumber fourBarMinDegrees = new TunableNumber("Intake/Four Bar/Min Degrees", kFourBarMinAngle.in(Degrees));
    public static final TunableNumber fourBarMaxDegrees = new TunableNumber("Intake/Four Bar/Max Degrees", kFourBarMaxAngle.in(Degrees));
    public static final TunableNumber degreeTolerance = new TunableNumber("Intake/Four Bar/Degrees Tolerance", kAngleTolerance.in(Degrees));

    public static final double kIntakeVoltageIn = 4; //TODO: Tune
    public static final double kIntakeVoltageOut = -4; //TODO: Tune

    public static final double kFourBarVoltageIn = 2; //TODO: Tune
    public static final double kFourBarVoltageOut = -2; //TODO: Tune

    public static final TunableNumber intakeVoltageIn = new TunableNumber("Intake/Roller/Voltage In", kIntakeVoltageIn);
    public static final TunableNumber intakeVoltageOut = new TunableNumber("Intake/Roller/Voltage Out", kIntakeVoltageOut);
    public static final TunableNumber fourBarVoltageIn = new TunableNumber("Intake/Four Bar/Voltage In", kFourBarVoltageIn);
    public static final TunableNumber fourBarVoltageOut = new TunableNumber("Intake/Four Bar/Voltage Out", kFourBarVoltageOut);

    public static final MomentOfInertia kMomentOfInertia = MomentOfInertia.ofRelativeUnits(10, PoundSquareInches);//TODO: tune

    public static final TalonFXConfiguration kIntakeConfig = new TalonFXConfiguration();
    static { 
        FeedbackConfigs feedback = kIntakeConfig.Feedback;
        feedback.SensorToMechanismRatio = kIntakeGearRatio;

        MotorOutputConfigs output = kIntakeConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Coast;
        output.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs current = kIntakeConfig.CurrentLimits;
        current.StatorCurrentLimitEnable = true;
        current.StatorCurrentLimit = 40; 

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
        current.StatorCurrentLimit = 40; 

        Slot0Configs control = kFourBarConfig.Slot0;// TODO: Tune PID
            control.kP = 0.1; 
            control.kI = 0;
            control.kD = 0;
            
            control.kS = 0;
            control.kV = 0;
            control.kA = 0;

            
    }
}
