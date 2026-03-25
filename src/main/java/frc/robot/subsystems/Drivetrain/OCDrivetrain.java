package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static frc.robot.util.RobotConstants.kPigeonID;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;
import frc.robot.util.RobotConstants;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableUnits.TunableAngularAcceleration;
import frc.robot.util.TunableUnits.TunableLinearAcceleration;

public class OCDrivetrain extends CommandSwerveDrivetrain {

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // max angular velocity

    // Normal driving speed at 100% controller input
    public static final double kDriveSpeedRatio = 0.9;
    public static final double kTurnSpeedRatio = 0.5;

    public static final TunableNumber driveSpeedRatio = new TunableNumber("1) Drivetrain/Drive Speed", kDriveSpeedRatio);
    public static final TunableNumber turnSpeedRatio = new TunableNumber("1) Drivetrain/Turn Speed", kTurnSpeedRatio);

    // Normal driving acceleration limits
    public static final LinearAcceleration kLinearAccel = FeetPerSecondPerSecond.of(35);
    public static final LinearAcceleration kLinearDecel = FeetPerSecondPerSecond.of(50);
    public static final AngularAcceleration kAngularAccel = RotationsPerSecondPerSecond.of(6);
    public static final AngularAcceleration kAngularDecel = RotationsPerSecondPerSecond.of(10);

    public static final TunableLinearAcceleration linearAccel = new TunableLinearAcceleration("1) Drivetrain/Linear Acceleration", kLinearAccel);
    public static final TunableLinearAcceleration linearDecel = new TunableLinearAcceleration("1) Drivetrain/Linear Deceleration", kLinearDecel);
    public static final TunableAngularAcceleration angularAccel = new TunableAngularAcceleration("1) Drivetrain/Angular Acceleration", kAngularAccel);
    public static final TunableAngularAcceleration angularDecel = new TunableAngularAcceleration("1) Drivetrain/Angular Deceleration", kAngularDecel);

    public final SwerveDriveLimiter kStandardLimiter = new SwerveDriveLimiter(
            getDriveSpeed(),
            linearAccel.get(),
            linearDecel.get(),
            getTurnSpeed(),
            angularAccel.get(),
            angularAccel.get());

    // private final SwerveModule[] swerveMods = {
    // new SwerveModule(SwerveConstants.Module.FL),
    // new SwerveModule(SwerveConstants.Module.FR),
    // new SwerveModule(SwerveConstants.Module.BL),
    // new SwerveModule(SwerveConstants.Module.BR)
    // };

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveautos = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle face = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(6.5, 0, 0); // TODO: tune PID
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public ChassisSpeeds lastTargetSpeeds = new ChassisSpeeds();
    private final Pigeon2 gyro = new Pigeon2(kPigeonID);

    public final SwerveDrivePoseEstimator visionEstimator = new SwerveDrivePoseEstimator(
            getKinematics(),
            getState().Pose.getRotation(),
            getState().ModulePositions,
            getState().Pose);

    private final StructPublisher<Pose2d> estimatedPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("Swerve/Estimated Pose", Pose2d.struct).publish();


    public OCDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
    }

    @Override
    public void periodic() {
        changeTunable();

        double phoenixTimeOffset = Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
        var state = getState();
        visionEstimator.updateWithTime(state.Timestamp + phoenixTimeOffset, state.RawHeading, state.ModulePositions);
        estimatedPosePub.set(visionEstimator.getEstimatedPosition());
        log();

    }

    public LinearVelocity getDriveSpeed() {
        return MetersPerSecond.of(driveSpeedRatio.get() * MaxSpeed);
    }

    public AngularVelocity getTurnSpeed() {
        return RadiansPerSecond.of(turnSpeedRatio.get() * MaxAngularRate);
    }

    public Pose2d getGlobalPoseEstimate() {
        return visionEstimator.getEstimatedPosition();
    }

    public Optional<Pose2d> sampleGlobalPoseEstimateAt(double timestampSeconds) {
        return visionEstimator.sampleAt(timestampSeconds);
    }

    public Command driveC(OCXboxController controller) {
        return applyRequest(() -> {
            ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(controller.getSpeeds(MaxSpeed, MaxAngularRate),
                    lastTargetSpeeds, Robot.kDefaultPeriod);
            lastTargetSpeeds = targetSpeeds;
            return drive.withVelocityX(targetSpeeds.vxMetersPerSecond)
                .withVelocityY(targetSpeeds.vyMetersPerSecond)
                .withRotationalRate(targetSpeeds.omegaRadiansPerSecond);
        });
    }

    public Command driveC(ChassisSpeeds chassisSpeeds) {
        return applyRequest(() -> {
            ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(chassisSpeeds, lastTargetSpeeds, //why do we call it last target speeds
                    Robot.kDefaultPeriod);
            lastTargetSpeeds = targetSpeeds;
            return drive.withVelocityX(targetSpeeds.vxMetersPerSecond)
                .withVelocityY(targetSpeeds.vyMetersPerSecond)
                .withRotationalRate(targetSpeeds.omegaRadiansPerSecond);
        });
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(chassisSpeeds, lastTargetSpeeds, //why do we call it last target speeds
                Robot.kDefaultPeriod);
        lastTargetSpeeds = targetSpeeds;
        setControl(
            drive.withVelocityX(targetSpeeds.vxMetersPerSecond)
            .withVelocityY(targetSpeeds.vyMetersPerSecond)
            .withRotationalRate(targetSpeeds.omegaRadiansPerSecond)
        );
    }

    public void driveAutos(ChassisSpeeds chassisSpeeds) {
        setControl(
            driveautos.withVelocityX(chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond)
        );
    }

    public Command faceAngle(Angle angle) {
        return applyRequest(() -> face.withTargetDirection(Rotation2d.fromDegrees(angle.in(Degrees))));
    }

    public Command driveFacingHubController(Supplier<OCXboxController> controller) {
        return driveFacingHub(controllerToChassisSpeeds(controller));
    }
    
    public Command driveFacingHub(Supplier<ChassisSpeeds> speeds) {
        return driveFacingTargetC(speeds, () -> FieldUtil.kHubTrl);
    }

    public Command driveFacingSetpoint(Supplier<ChassisSpeeds> speeds) {
        return driveFacingTargetC(speeds, () -> getGlobalPoseEstimate().nearest(FieldUtil.kSetpoints).getTranslation());
    }

    public Command driveFacingTargetC(Supplier<ChassisSpeeds> speeds, Supplier<Translation2d> target) {
        return applyRequest(() -> {
            ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(speeds.get(), lastTargetSpeeds, Robot.kDefaultPeriod);
            lastTargetSpeeds = targetSpeeds;
            return face.withVelocityX(targetSpeeds.vxMetersPerSecond)
                    .withVelocityY(targetSpeeds.vyMetersPerSecond)
                    .withTargetDirection(Shotmap.newTargetAngle(getGlobalPoseEstimate().plus(new Transform2d(RobotConstants.kShooterTranslation, Rotation2d.kZero)), targetSpeeds, target.get()).plus(Rotation2d.k180deg));
        });
    }

    public void driveFacingTarget(Supplier<ChassisSpeeds> speeds, Supplier<Translation2d> target) {
        ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(speeds.get(), lastTargetSpeeds, Robot.kDefaultPeriod);
        lastTargetSpeeds = targetSpeeds;
        setControl(
            face.withVelocityX(targetSpeeds.vxMetersPerSecond)
                    .withVelocityY(targetSpeeds.vyMetersPerSecond)
                    .withTargetDirection(Shotmap.newTargetAngle(getGlobalPoseEstimate().plus(new Transform2d(RobotConstants.kShooterTranslation, Rotation2d.kZero)), targetSpeeds, target.get()).plus(Rotation2d.k180deg))
        );
    } 

    public void driveFacingAngle(Supplier<ChassisSpeeds> speeds, Angle target) {
        ChassisSpeeds targetSpeeds = kStandardLimiter.calculate(speeds.get(), lastTargetSpeeds, Robot.kDefaultPeriod);
        lastTargetSpeeds = targetSpeeds;
        setControl(
            face.withVelocityX(targetSpeeds.vxMetersPerSecond)
                    .withVelocityY(targetSpeeds.vyMetersPerSecond)
                    .withTargetDirection(Rotation2d.fromDegrees(target.in(Degrees))//.plus(Rotation2d.k180deg)
        ));
    }

    public Command driveFacingOptionalTarget(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target) {
        return either(
            runOnce(()-> drive(speeds.get())), 
            runOnce(()-> driveFacingTarget(speeds, ()-> target.get().get())), 
            ()-> target.get().isEmpty()
        ).repeatedly();
    }

    public Trigger facingTargetT(Supplier<Translation2d> trl) {
        return new Trigger(() -> getGlobalPoseEstimate().getTranslation().minus(trl.get()).plus(RobotConstants.kShooterTranslation).getAngle()
                .getDegrees() == getGlobalPoseEstimate().getRotation().getDegrees())
                .debounce(0.25);// TODO: Tune
    }

    public Command brakeC(){
        return applyRequest(() -> brake);
    }

    public Trigger inAllianceZone() {
        return new Trigger(() -> {
            Pose2d botTrl = getGlobalPoseEstimate();
            return botTrl.getX() <= FieldUtil.kAllianceZoneMax.getX() &&
                botTrl.getY() <= FieldUtil.kAllianceZoneMax.getY();
            }
        );
    }    

    public Trigger inTrenchZone() {
        return new Trigger(() -> {
            Pose2d botTrl = getGlobalPoseEstimate();
            boolean inBottomTrench = botTrl.getX() >= FieldUtil.kBottomTrenchZoneMin.getX()
                && botTrl.getX() <= FieldUtil.kBottomTrenchZoneMax.getX()
                && botTrl.getY() >= FieldUtil.kBottomTrenchZoneMin.getY()
                && botTrl.getY() <= FieldUtil.kBottomTrenchZoneMax.getY();

            boolean inTopTrench = botTrl.getX() >= FieldUtil.kTopTrenchZoneMin.getX()
                && botTrl.getX() <= FieldUtil.kTopTrenchZoneMax.getX()
                && botTrl.getY() >= FieldUtil.kTopTrenchZoneMin.getY()
                && botTrl.getY() <= FieldUtil.kTopTrenchZoneMax.getY();

            return inBottomTrench || inTopTrench;
            }
        );
    } 

    public Trigger inNeutralZone() {
        return new Trigger(() -> {
            Pose2d botTrl = getGlobalPoseEstimate();
            return botTrl.getX() >= FieldUtil.kNeutralZoneMin.getX() &&
                botTrl.getX() <= FieldUtil.kNeutralZoneMax.getX() &&
                botTrl.getY() >= FieldUtil.kNeutralZoneMin.getY() &&
                botTrl.getY() <= FieldUtil.kNeutralZoneMax.getY();
            }
        );
    }

    public static Supplier<ChassisSpeeds> controllerToChassisSpeeds(Supplier<OCXboxController> controller) {
        return ()->controller.get().getSpeeds(MaxSpeed, MaxAngularRate);
    }

    public void disturbSimPose() {
        var disturbance = new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
        super.resetPose(getState().Pose.plus(disturbance));
    }

    public void disturbGlobalPoseEstimate() {
        var disturbance = new Transform2d(new Translation2d(Math.random(), Math.random()), Rotation2d.kZero);
        visionEstimator.resetPose(getGlobalPoseEstimate().plus(disturbance));
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        visionEstimator.resetPose(pose);
        // visionEstimator.resetTranslation(pose.getTranslation());
    }

    @Override
    public void resetTranslation(Translation2d translation) {
        super.resetTranslation(translation);
        visionEstimator.resetTranslation(translation);
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        super.resetRotation(rotation);
        visionEstimator.resetRotation(rotation);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        visionEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void resetOdometry(Pose2d pose) {
        visionEstimator.resetPosition(getGyroYaw(), getState().ModulePositions, pose);
    }

    public SwerveRequest.FieldCentric getDriveRequest() {
        return drive;
    }

    // public SwerveModulePosition getPosition() {
    // return new SwerveModulePosition(
    // Utils.positionToMeters(driveMotor.position(), kDriveGearRatio,
    // kWheelCircumference),
    // getAbsoluteHeading()
    // );
    // }

    // public SwerveModulePosition[] getModulePositions() {
    // return new SwerveModulePosition[] {
    // new SwerveModulePosition(
    // FrontLeft.getPosition(), // meters
    // Rotation2d.fromRadians(FrontLeft.getPosition())
    // ),

    // new SwerveModulePosition(
    // FrontRight.getPosition(),
    // Rotation2d.fromRadians(FrontRight.getPosition())
    // ),

    // new SwerveModulePosition(
    // BackLeft.getPosition(),
    // Rotation2d.fromRadians(BackLeft.getPosition())
    // ),

    // new SwerveModulePosition(
    // BackLeft.getPosition,
    // Rotation2d.fromRadians(BackRight.getPosition())
    // )
    // };
    // }

    public Command resetInitialOdomC() {
        return runOnce(() -> {
            Rotation2d initialRot = new Rotation2d();
            if (driveMirror()) {
                initialRot = new Rotation2d(Math.PI);
            }
            resetOdometry(
                    new Pose2d(
                            getState().Pose.getTranslation(),
                            initialRot));
        });
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getRoll().getValueAsDouble());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getPitch().getValueAsDouble());
    }

    public boolean driveMirror() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        visionEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public void changeTunable() {
        driveSpeedRatio.poll();
        turnSpeedRatio.poll();
        linearAccel.poll();
        linearDecel.poll();
        angularAccel.poll();
        angularDecel.poll();

        int hash = hashCode();

        // Standard limiter
        if (driveSpeedRatio.hasChanged(hash) || turnSpeedRatio.hasChanged(hash) || linearAccel.hasChanged(hash)
                || linearDecel.hasChanged(hash) || angularAccel.hasChanged(hash) || angularDecel.hasChanged(hash)) {
            kStandardLimiter.linearTopSpeed = getDriveSpeed();
            kStandardLimiter.angularTopSpeed = getTurnSpeed();
            kStandardLimiter.linearAcceleration = linearAccel.get();
            kStandardLimiter.linearDeceleration = linearDecel.get();
            kStandardLimiter.angularAcceleration = angularAccel.get();
            kStandardLimiter.angularDeceleration = angularDecel.get();
        }
    }

    private void log() {
        SmartDashboard.putBoolean("1) Drivetrain/In Trench Zone", inTrenchZone().getAsBoolean());
        SmartDashboard.putBoolean("1) Drivetrain/In Aliiance Zone", inAllianceZone().getAsBoolean());
        SmartDashboard.putBoolean("1) Drivetrain/In Neutral Zone", inNeutralZone().getAsBoolean());
        var state = getState();
        if (state != null && state.Pose != null) {
            Rotation2d targetAngle = Shotmap.newTargetAngle(getGlobalPoseEstimate(), lastTargetSpeeds, FieldUtil.kHubTrl);
            Rotation2d rawAngle = FieldUtil.kHubTrl.minus(getGlobalPoseEstimate().getTranslation()).getAngle();

            SmartDashboard.putNumber("1) Drivetrain/DriveFacingHub/TargetAngleDeg", targetAngle.plus(Rotation2d.k180deg).getDegrees());
            SmartDashboard.putNumber("1) Drivetrain/DriveFacingHub/RotationDeg", getGlobalPoseEstimate().getRotation().getDegrees());
            SmartDashboard.putNumber("1) Drivetrain/DriveFacingHub/RawAngleDeg", rawAngle.plus(Rotation2d.k180deg).getDegrees());
            SmartDashboard.putNumber("1) Drivetrain/Dist from hub", Shotmap.distanceToHub(getGlobalPoseEstimate()).in(Meters));
        }
    }
}
