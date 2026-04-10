package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableUnits.TunableLinearAcceleration;
import frc.robot.util.TunableUnits.TunableTime;

public class DrivetrainConstants {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity

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


}