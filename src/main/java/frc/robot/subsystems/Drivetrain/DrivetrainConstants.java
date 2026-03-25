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

    public static final TunableNumber driveSpeedRatio = new TunableNumber("1) Drivetrain/Standard/Drive Speed", kDriveSpeedRatio);
    public static final TunableNumber turnSpeedRatio = new TunableNumber("1) Drivetrain/Standard/Turn Speed", kTurnSpeedRatio);



    // Normal driving acceleration limits
    public static final double kLinearAccel = FeetPerSecondPerSecond.of(35).in(MetersPerSecondPerSecond);
    public static final double kLinearDecel = FeetPerSecondPerSecond.of(50).in(MetersPerSecondPerSecond);
    public static final double kAngularAccel = RotationsPerSecondPerSecond.of(6).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecel = RotationsPerSecondPerSecond.of(10).in(RadiansPerSecondPerSecond);

    public static final TunableNumber linearAccel = new TunableNumber("1) Drivetrain/Standard/Linear Acceleration", kLinearAccel);
    public static final TunableNumber linearDecel = new TunableNumber("1) Drivetrain/Standard/Linear Deceleration", kLinearDecel);
    public static final TunableNumber angularAccel = new TunableNumber("1) Drivetrain/Standard/Angular Acceleration", kAngularAccel);
    public static final TunableNumber angularDecel = new TunableNumber("1) Drivetrain/Standard/Angular Deceleration", kAngularDecel);

    public static final double kSOTMDriveSpeedRatio = 0.56;

    public static final TunableNumber sotmDriveSpeedRatio = new TunableNumber("1) Drivetrain/SOTM/Drive Speed", kSOTMDriveSpeedRatio);

    public static final double kSOTMLinearAccel = FeetPerSecondPerSecond.of(25).in(MetersPerSecondPerSecond);
    public static final double kSOTMLinearDecel = FeetPerSecondPerSecond.of(35).in(MetersPerSecondPerSecond);

    public static final TunableNumber sotmLinearAccel = new TunableNumber("1) Drivetrain/SOTM/Linear Acceleration", kSOTMLinearAccel);
    public static final TunableNumber sotmLinearDecel = new TunableNumber("1) Drivetrain/SOTM/Linear Deceleration", kSOTMLinearDecel);
    
    public static final Angle kRotationToleranceDegrees = Degrees.of(2);
    public static final TunableNumber rotationToleranceDegrees = new TunableNumber("1) Drivetrain/Rotation Tolerance", kRotationToleranceDegrees.in(Degrees));

    public static final Time kRotationDebounceTime = Seconds.of(0.2);
    public static final TunableNumber rotationDebounceSeconds = new TunableNumber("1) Drivetrain/Rotation Debounce", kRotationDebounceTime.in(Seconds));
   
    // public static final Time kBrakeDebounceTime = Seconds.of(0.2);
    // public static final TunableNumber brakeDebounceSeconds = new TunableNumber("1) Drivetrain/Brake Debounce", kBrakeDebounceTime.in(Seconds));


}