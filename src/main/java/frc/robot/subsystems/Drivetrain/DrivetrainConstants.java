package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.TunableNumber;

public class DrivetrainConstants {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity

    // Normal driving speed at 100% controller input
    public static final double kDriveSpeedRatio = 0.9;
    public static final double kTurnSpeedRatio = 0.5;

    public static final TunableNumber driveSpeedRatio = new TunableNumber("1) Drivetrain/Drive Speed", kDriveSpeedRatio);
    public static final TunableNumber turnSpeedRatio = new TunableNumber("1) Drivetrain/Turn Speed", kTurnSpeedRatio);

    // Normal driving acceleration limits
    public static final double kLinearAccel = FeetPerSecondPerSecond.of(35).in(MetersPerSecondPerSecond);
    public static final double kLinearDecel = FeetPerSecondPerSecond.of(50).in(MetersPerSecondPerSecond);
    public static final double kAngularAccel = RotationsPerSecondPerSecond.of(6).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecel = RotationsPerSecondPerSecond.of(10).in(RadiansPerSecondPerSecond);

    public static final TunableNumber linearAccel = new TunableNumber("1) Drivetrain/Linear Acceleration", kLinearAccel);
    public static final TunableNumber linearDecel = new TunableNumber("1) Drivetrain/Linear Deceleration", kLinearDecel);
    public static final TunableNumber angularAccel = new TunableNumber("1) Drivetrain/Angular Acceleration", kAngularAccel);
    public static final TunableNumber angularDecel = new TunableNumber("1) Drivetrain/Angular Deceleration", kAngularDecel);

    public static final Angle kRotationToleranceDegrees = Degrees.of(2);
    public static final TunableNumber rotationToleranceDegrees = new TunableNumber("1) Drivetrain/Rotation Tolerance", kRotationToleranceDegrees.in(Degrees));

    public static final Time kRotationDebounceTime = Seconds.of(0.2);
    public static final TunableNumber RotationDebounceSeconds = new TunableNumber("1) Drivetrain/Rotation Debounce", kRotationDebounceTime.in(Seconds));
}