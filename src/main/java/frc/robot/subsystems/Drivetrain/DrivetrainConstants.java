package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableUnits.TunableAngularVelocity;
import frc.robot.util.TunableUnits.TunableLinearAcceleration;
import frc.robot.util.TunableUnits.TunableTime;

public class DrivetrainConstants {
    public static double kMaxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    public static double kMaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity

    // Normal driving speed at 100% controller input
    public static final double kDriveSpeedRatio = 0.9;
    public static final double kTurnSpeedRatio = 0.5;

    // public static final TunableNumber driveSpeedRatio = new TunableNumber("1) Drivetrain/Standard/Drive Speed", kDriveSpeedRatio);
    // public static final TunableNumber turnSpeedRatio = new TunableNumber("1) Drivetrain/Standard/Turn Speed", kTurnSpeedRatio);

    // Normal driving acceleration limits
    public static final LinearAcceleration kLinearAccel = FeetPerSecondPerSecond.of(40);
    public static final LinearAcceleration kLinearDecel = FeetPerSecondPerSecond.of(60);
    public static final AngularAcceleration kAngularAccel = RotationsPerSecondPerSecond.of(6);
    public static final AngularAcceleration kAngularDecel = RotationsPerSecondPerSecond.of(10);

    // public static final TunableLinearAcceleration linearAccel = new TunableLinearAcceleration("1) Drivetrain/Linear Acceleration", kLinearAccel);
    // public static final TunableLinearAcceleration linearDecel = new TunableLinearAcceleration("1) Drivetrain/Linear Deceleration", kLinearDecel);
    // public static final TunableAngularAcceleration angularAccel = new TunableAngularAcceleration("1) Drivetrain/Angular Acceleration", kAngularAccel);
    // public static final TunableAngularAcceleration angularDecel = new TunableAngularAcceleration("1) Drivetrain/Angular Deceleration", kAngularDecel);

    public static final double kSOTMDriveSpeedRatio = 0.15;

    public static final TunableNumber sotmDriveSpeedRatio = new TunableNumber("1) Drivetrain/SOTM/Drive Speed", kSOTMDriveSpeedRatio);

    public static final Time kSOTMLatency = Seconds.of(0.1);
    public static final TunableTime SOTMLatency = new TunableTime("1) Drivetrain/SOTM/Latency", kSOTMLatency);    

    public static final LinearAcceleration kSOTMLinearAccel = FeetPerSecondPerSecond.of(10);
    public static final LinearAcceleration kSOTMLinearDecel = FeetPerSecondPerSecond.of(15);

    public static final TunableLinearAcceleration sotmLinearAccel = new TunableLinearAcceleration("1) Drivetrain/SOTM/Linear Acceleration", kSOTMLinearAccel);
    public static final TunableLinearAcceleration sotmLinearDecel = new TunableLinearAcceleration("1) Drivetrain/SOTM/Linear Deceleration", kSOTMLinearDecel);
    
    public static final Angle kRotationTolerance = Degrees.of(4);
    // public static final TunableAngle rotationTolerance = new TunableAngle("1) Drivetrain/Rotation Tolerance", kRotationTolerance);

    public static final AngularVelocity kOmegaTolerance = DegreesPerSecond.of(5);
    // public static final TunableAngularVelocity omegaTolerance = new TunableAngularVelocity("1) Drivetrain/Omega Tolerance", kOmegaTolerance);

    public static final Time kRotationDebounceTime = Seconds.of(0.3);
    public static final TunableTime rotationDebounceTime = new TunableTime("1) Drivetrain/Rotation Debounce", kRotationDebounceTime);
   
    // public static final Time kBrakeDebounceTime = Seconds.of(0.2);
    // public static final TunableTime brakeDebounceSeconds = new TunableTime("1) Drivetrain/Brake Debounce", kBrakeDebounceTime);

    public static final AngularVelocity kRotationalKs = DegreesPerSecond.of(0);
    public static final TunableAngularVelocity rotationalKs = new TunableAngularVelocity("1) Drivetrain/Rotational Ks (DegreesPerSecond)", kRotationalKs);
    
    // Path following constants
    public static final double kPathDriveKP = 3;
    public static final double kPathDriveKI = 0;
    public static final double kPathDriveKD = 0;
    
    public static final double kPathTurnKP = 3;
    public static final double kPathTurnKI = 0;
    public static final double kPathTurnKD = 0;

    // Driving speed for auto-alignment
    public static final double kDriveSpeedAlign = 0.6 * kMaxLinearSpeed;
    public static final double kTurnSpeedAlign = 0.5 * kMaxAngularRate;
    // Driving acceleration for auto-alignment
    public static final double kLinearAccelAlign = FeetPerSecondPerSecond.of(25).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kAngularAccelAlign = RotationsPerSecondPerSecond.of(4).in(RadiansPerSecondPerSecond);

    public static final SwerveDriveLimiter kAlignLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(kDriveSpeedAlign),
        MetersPerSecondPerSecond.of(kLinearAccelAlign),
        MetersPerSecondPerSecond.of(kLinearAccelAlign),
        RadiansPerSecond.of(kTurnSpeedAlign),
        RadiansPerSecondPerSecond.of(kAngularAccelAlign),
        RadiansPerSecondPerSecond.of(kAngularAccelAlign)
    );

    // Threshold to use final alignment speeds
    public static final Distance kFinalAlignDistReef = Feet.of(3);
    public static final Distance kFinalAlignDistStation = Feet.of(2);

    // Driving speed for final auto-alignment
    public static final double kDriveSpeedAlignFinal = 0.25 * kMaxLinearSpeed;
    public static final double kTurnSpeedAlignFinal = 0.2 * kMaxAngularRate;
    // Driving acceleration for final auto-alignment
    public static final double kLinearAccelAlignFinal = FeetPerSecondPerSecond.of(8).in(MetersPerSecondPerSecond); //m/s/s
    public static final double kAngularAccelAlignFinal = RotationsPerSecondPerSecond.of(4).in(RadiansPerSecondPerSecond);

    public static final SwerveDriveLimiter kAlignFinalLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(kDriveSpeedAlignFinal),
        MetersPerSecondPerSecond.of(kLinearAccelAlignFinal),
        MetersPerSecondPerSecond.of(kLinearAccelAlignFinal),
        RadiansPerSecond.of(kTurnSpeedAlignFinal),
        RadiansPerSecondPerSecond.of(kAngularAccelAlignFinal),
        RadiansPerSecondPerSecond.of(kAngularAccelAlignFinal)
    );

    public static final double kAlignDrivePosTol = Inches.of(0.8).in(Meters);
    public static final double kAlignDriveVelTol = Inches.of(2.5).in(Meters);

    public static final double kAlignTurnPosTol = Degrees.of(2.5).in(Radians);
    public static final double kAlignTurnVelTol = Degrees.of(4).in(Radians);

    // Threshold to output zero when close
    public static final double kStopAlignTrlDist = Inches.of(0.6).in(Meters);
    public static final double kStopAlignRotDist = Degrees.of(2).in(Radians);
}