package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;

public class OCDrivetrain extends CommandSwerveDrivetrain{
    
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity
    
    // Normal driving speed at 100% controller input
    public static final double kDriveSpeed = 0.5 * MaxSpeed;
    public static final double kTurnSpeed = 0.5 * MaxAngularRate;

    // Normal driving acceleration limits
    public static final double kLinearAccel = FeetPerSecondPerSecond.of(30).in(MetersPerSecondPerSecond);
    public static final double kLinearDecel = FeetPerSecondPerSecond.of(40).in(MetersPerSecondPerSecond);
    public static final double kAngularAccel = RotationsPerSecondPerSecond.of(6).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecel = RotationsPerSecondPerSecond.of(10).in(RadiansPerSecondPerSecond);
    
    public static final SwerveDriveLimiter kStandardLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(kDriveSpeed),
        MetersPerSecondPerSecond.of(kLinearAccel),
        MetersPerSecondPerSecond.of(kLinearDecel),
        RadiansPerSecond.of(kTurnSpeed),
        RadiansPerSecondPerSecond.of(kAngularAccel),
        RadiansPerSecondPerSecond.of(kAngularDecel)
    );

    ChassisSpeeds lastTargetSpeeds = new ChassisSpeeds();
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentricFacingAngle face = new SwerveRequest.RobotCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(3, 0, 0); // TODO: tune PID
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    public OCDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
    }

    public Command drive(OCXboxController controller){
        return applyRequest(() -> {
            ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(controller.getSpeeds(MaxSpeed, MaxAngularRate), lastTargetSpeeds, Robot.kDefaultPeriod);
            lastTargetSpeeds = targetSpeeds;
            return drive.withVelocityX(-targetSpeeds.vxMetersPerSecond)
                        .withVelocityY(-targetSpeeds.vyMetersPerSecond)
                        .withRotationalRate(targetSpeeds.omegaRadiansPerSecond);
        });
    }
    
    public Command faceAngle(Angle angle){
        return applyRequest(() -> face.withTargetDirection(Rotation2d.fromDegrees(angle.in(Degrees))));
    }
    
    public Command faceHub(){
        return applyRequest(() -> face.withTargetDirection(getState().Pose.getTranslation().minus(FieldUtil.kHubTrl).getAngle())); // TODO: (maybe) switch directions €
    }

    public Command driveFacingHub(OCXboxController controller){
        return applyRequest(() -> {
            ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(controller.getSpeeds(MaxSpeed, MaxAngularRate), lastTargetSpeeds, Robot.kDefaultPeriod);
            lastTargetSpeeds = targetSpeeds;
            return face.withVelocityX(-targetSpeeds.vxMetersPerSecond)
                        .withVelocityY(-targetSpeeds.vyMetersPerSecond)
                        .withTargetDirection(getState().Pose.getTranslation().minus(FieldUtil.kHubTrl).getAngle()); 
        });
    }

    public Trigger facingHubT() {
        return new Trigger(() -> getState().Pose.getTranslation().minus(FieldUtil.kHubTrl).getAngle().getDegrees() == getState().Pose.getRotation().getDegrees())
                                    .debounce(0.25);// TODO: Tune
    }
    
}
