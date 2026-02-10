package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;
import frc.robot.util.TunableNumber;

public class OCDrivetrain extends CommandSwerveDrivetrain{
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity

    // Normal driving speed at 100% controller input
    public static final double kDriveSpeedRatio = 0.5;
    public static final double kTurnSpeedRatio = 0.5;

    public static final TunableNumber driveSpeedRatio = new TunableNumber("Drivetrain/Drive Speed", kDriveSpeedRatio);
    public static final TunableNumber turnSpeedRatio = new TunableNumber("Drivetrain/Turn Speed", kTurnSpeedRatio);

    // Normal driving acceleration limits
    public static final double kLinearAccel = FeetPerSecondPerSecond.of(30).in(MetersPerSecondPerSecond);
    public static final double kLinearDecel = FeetPerSecondPerSecond.of(40).in(MetersPerSecondPerSecond);
    public static final double kAngularAccel = RotationsPerSecondPerSecond.of(6).in(RadiansPerSecondPerSecond);
    public static final double kAngularDecel = RotationsPerSecondPerSecond.of(10).in(RadiansPerSecondPerSecond);
    
    public static final TunableNumber linearAccel = new TunableNumber("Drivetrain/Linear Acceleration", kLinearAccel);
    public static final TunableNumber linearDecel = new TunableNumber("Drivetrain/Linear Deceleration", kLinearDecel);
    public static final TunableNumber angularAccel = new TunableNumber("Drivetrain/Angular Acceleration", kAngularAccel);
    public static final TunableNumber angularDecel = new TunableNumber("Drivetrain/Angular Deceleration", kAngularDecel);

    public final SwerveDriveLimiter kStandardLimiter = new SwerveDriveLimiter(
        MetersPerSecond.of(getDriveSpeed()),
        MetersPerSecondPerSecond.of(linearAccel.get()),
        MetersPerSecondPerSecond.of(linearDecel.get()),
        RadiansPerSecond.of(getTurnSpeed()),
        RadiansPerSecondPerSecond.of(angularAccel.get()),
        RadiansPerSecondPerSecond.of(angularAccel.get())
    );
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle face = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(9, 0, 0); // TODO: tune PID
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public ChassisSpeeds lastTargetSpeeds = new ChassisSpeeds();
    
    public OCDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
    }

    public double getDriveSpeed(){
        return driveSpeedRatio.get() * MaxSpeed;
    }

    public double getTurnSpeed(){
        return turnSpeedRatio.get() * MaxAngularRate;
    }

    public Pose2d getPose() {
        Optional<Pose2d> pose = samplePoseAt(0);
        return pose.orElse(new Pose2d());
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
        return applyRequest(() -> {
            Translation2d safeTarget = clampToHubBuffer(getState().Pose.getTranslation());
            return face.withTargetDirection(getState().Pose.getTranslation().minus(safeTarget).getAngle());
        }); // TODO: (maybe) switch directions 
    }

    public Command driveFacingHub(OCXboxController controller){
        return applyRequest(() -> {
            ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(controller.getSpeeds(MaxSpeed, MaxAngularRate), lastTargetSpeeds, Robot.kDefaultPeriod);
                lastTargetSpeeds = targetSpeeds;

                // Cap motion so we never go inside hub
                targetSpeeds = hubBarrier(targetSpeeds, getState().Pose);

                return face.withVelocityX(-targetSpeeds.vxMetersPerSecond)
                        .withVelocityY(-targetSpeeds.vyMetersPerSecond)
                        .withTargetDirection(getState().Pose.getTranslation().minus(FieldUtil.kHubTrl).getAngle());
            });
    }

    public Trigger facingHubT() {
        return new Trigger(() -> getState().Pose.getTranslation().minus(FieldUtil.kHubTrl).getAngle().getDegrees() == getState().Pose.getRotation().getDegrees())
                                    .debounce(0.25);// TODO: Tune
    }

    private ChassisSpeeds hubBarrier(ChassisSpeeds speeds, Pose2d pose) {
        Translation2d robotPos = pose.getTranslation();
        Translation2d hubPos = FieldUtil.kHubTrl;

        double dx = hubPos.getX() - robotPos.getX();
        double dy = hubPos.getY() - robotPos.getY();
        double distance = Math.hypot(dx, dy);

        if (distance < FieldUtil.kHubBarrier.in(Meters)) {
            double angleToHub = Math.atan2(dy, dx);

            double vAlong = speeds.vxMetersPerSecond * Math.cos(angleToHub) +
                            speeds.vyMetersPerSecond * Math.sin(angleToHub);

            if (vAlong > 0) {
                double newVx = speeds.vxMetersPerSecond - vAlong * Math.cos(angleToHub);
                double newVy = speeds.vyMetersPerSecond - vAlong * Math.sin(angleToHub);
                return new ChassisSpeeds(newVx, newVy, speeds.omegaRadiansPerSecond);
            }
        }

        return speeds;
    }


        
    private Translation2d clampToHubBuffer(Translation2d robotPos) {
        double hubX = FieldUtil.kHubTrl.getX();
        double hubY = FieldUtil.kHubTrl.getY();
        double halfWidth = FieldUtil.kHubWidth.in(Meters) / 2.0 + FieldUtil.kHubBarrier.in(Meters);
        double halfHeight = FieldUtil.kHubWidth.in(Meters) / 2.0 + FieldUtil.kHubBarrier.in(Meters); // assuming square for simplicity

        double dx = robotPos.getX() - hubX;
        double dy = robotPos.getY() - hubY;

        if (Math.abs(dx) < halfWidth) {
            dx = Math.signum(dx) * halfWidth;
        }

        if (Math.abs(dy) < halfHeight) {
            dy = Math.signum(dy) * halfHeight;
        }

        return new Translation2d(hubX + dx, hubY + dy);
    }
    
    public void changeTunable(){
        driveSpeedRatio.poll();
        turnSpeedRatio.poll();
        linearAccel.poll();
        linearDecel.poll();
        angularAccel.poll();
        angularDecel.poll();
        
        int hash = hashCode();
        
        //Standard limiter
        if (driveSpeedRatio.hasChanged(hash) || turnSpeedRatio.hasChanged(hash) || linearAccel.hasChanged(hash) || linearDecel.hasChanged(hash) || angularAccel.hasChanged(hash) || angularDecel.hasChanged(hash)) {
            kStandardLimiter.linearTopSpeed = MetersPerSecond.of(getDriveSpeed());
            kStandardLimiter.angularTopSpeed = RadiansPerSecond.of(getTurnSpeed());
            kStandardLimiter.linearAcceleration = MetersPerSecondPerSecond.of(linearAccel.get());
            kStandardLimiter.linearDeceleration = MetersPerSecondPerSecond.of(linearDecel.get());
            kStandardLimiter.angularAcceleration = RadiansPerSecondPerSecond.of(angularAccel.get());
            kStandardLimiter.angularDeceleration = RadiansPerSecondPerSecond.of(angularDecel.get());
        }
    }
}
