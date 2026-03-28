package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Drivetrain.DrivetrainConstants.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import edu.wpi.first.units.measure.AngularVelocity;
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
import frc.robot.util.OCTrigger;

public class OCDrivetrain extends CommandSwerveDrivetrain {
    public final SwerveDriveLimiter kStandardLimiter = new SwerveDriveLimiter(
            getDriveSpeed(),
            linearAccel.get(),
            linearDecel.get(),
            getTurnSpeed(),
            angularAccel.get(),
            angularAccel.get());

    public final SwerveDriveLimiter kSOTMLimiter = new SwerveDriveLimiter(
            getSOTMDriveSpeed(),
            sotmLinearAccel.get(),
            sotmLinearDecel.get(),
            getTurnSpeed(),
            angularAccel.get(),
            angularAccel.get());

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

    public final SwerveDrivePoseEstimator visionEstimator = new SwerveDrivePoseEstimator(
            getKinematics(),
            getState().Pose.getRotation(),
            getState().ModulePositions,
            getState().Pose);

    private final StructPublisher<Pose2d> estimatedPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("Swerve/Estimated Pose", Pose2d.struct).publish();

    private final ChassisSpeeds kSpeedsZero = new ChassisSpeeds();
    public ChassisSpeeds lastTargetSpeeds = new ChassisSpeeds();
    public Angle targetRotation = Degrees.of(0);
    private double lastTargetTime = Timer.getFPGATimestamp();
    public AngularVelocity targetOmega = DegreesPerSecond.of(0);

    private double lastBrakeTime = Timer.getFPGATimestamp();

    private Angle lockAngle = Degrees.of(0);
    private double lastUnlockedTime = Timer.getFPGATimestamp();

    private Trigger isRotationTolerance = new Trigger(()-> {
        return getGlobalPoseEstimate().getRotation().getMeasure().isNear(targetRotation, rotationTolerance.get());
    });
    // TODO: to use this, we need to supply the target omega
    // (derived from the target translation and target chassis speeds while facing target)
    private Trigger isOmegaTolerance = new Trigger(()-> {
        return RadiansPerSecond.of(getState().Speeds.omegaRadiansPerSecond).isNear(targetOmega, omegaTolerance.get());
    });

    public final Trigger isBraking = new Trigger(() -> Timer.getFPGATimestamp() - lastBrakeTime < 0.04);

    public final Trigger turningToFaceTarget = new Trigger(()-> Timer.getFPGATimestamp() - lastTargetTime < 0.04);

    // TODO: omega tolerance
    public final Trigger isFacingTarget = turningToFaceTarget.or(isBraking).and(OCTrigger.debounce(isRotationTolerance, () -> rotationDebounceTime.in(Seconds), DebounceType.kBoth));
    
    // private Trigger driving = new Trigger(()-> driving())
    //             .debounce(brakeDebounceSeconds.get());


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

    public LinearVelocity getSOTMDriveSpeed() {
        return MetersPerSecond.of(sotmDriveSpeedRatio.get() * MaxSpeed);
    }

    public Pose2d getGlobalPoseEstimate() {
        return visionEstimator.getEstimatedPosition();
    }

    public Optional<Pose2d> sampleGlobalPoseEstimateAt(double timestampSeconds) {
        return visionEstimator.sampleAt(timestampSeconds);
    }

    public ChassisSpeeds limitTargetSpeeds(ChassisSpeeds targetSpeeds, SwerveDriveLimiter limiter) {
        var newSpeeds = limiter.calculate(targetSpeeds, lastTargetSpeeds, Robot.kDefaultPeriod);
        lastTargetSpeeds = newSpeeds;
        return newSpeeds;
    }

    public void drive(ChassisSpeeds targetSpeeds) {
        setControl(
            drive.withVelocityX(targetSpeeds.vxMetersPerSecond)
            .withVelocityY(targetSpeeds.vyMetersPerSecond)
            .withRotationalRate(targetSpeeds.omegaRadiansPerSecond)
        );
    }

    public void driveWithLock(ChassisSpeeds targetSpeeds) {
        double now = Timer.getFPGATimestamp();
        boolean stationary = targetSpeeds.equals(kSpeedsZero);
        if (stationary && now - lastUnlockedTime > 0.25) {
            driveFacingAngle(targetSpeeds, lockAngle);
        }
        else {
            if (!stationary){
                lastUnlockedTime = now;
            }
            lockAngle = Degrees.of(getState().Pose.getRotation().getDegrees());
            drive(targetSpeeds);
        }
    }

    public void driveAutos(ChassisSpeeds chassisSpeeds) {
        setControl(
            driveautos.withVelocityX(chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond)
        );
    }

    public Command driveC(Supplier<ChassisSpeeds> chassisSpeeds, boolean lockAngle) {
        return run(()-> {
            if (lockAngle) {
                driveWithLock(chassisSpeeds.get());
            }
            else {
                drive(chassisSpeeds.get());
            }
        });
    }

    public Command driveFacingTargetC(Supplier<ChassisSpeeds> speeds, Supplier<Translation2d> target) {
        return run(() -> driveFacingTarget(speeds.get(), target.get()));
    }

    public void driveFacingTarget(ChassisSpeeds speeds, Translation2d target) {
        lastTargetTime = Timer.getFPGATimestamp();
        // remove rotation input
        var targetSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
        targetRotation = Degrees.of(Shotmap.getFieldRelTargetFacingAngle(getGlobalPoseEstimate(), target).getDegrees());
        // TODO: add target omega
        driveFacingAngle(targetSpeeds, targetRotation);
    } 

    public void driveFacingAngle(ChassisSpeeds targetSpeeds, Angle target) {
        setControl(
            face.withVelocityX(targetSpeeds.vxMetersPerSecond)
                    .withVelocityY(targetSpeeds.vyMetersPerSecond)
                    .withTargetDirection(offsetTargetAngle(target))
        );
    }

    public Command driveFacingOptionalTargetC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target) {
        return run(()->driveFacingOptionalTarget(speeds.get(), target.get()));
    }

    public void driveFacingOptionalTarget(ChassisSpeeds speeds, Optional<Translation2d> target) {
        if(target.isEmpty()) {
            drive(speeds);
        }
        else {
            driveFacingTarget(speeds, target.get());
        }
    }

    public void driveFacingOptionalTargetBrake(ChassisSpeeds speeds, Optional<Translation2d> target) {
        boolean stationary = Math.abs(speeds.vxMetersPerSecond) < 0.01 && Math.abs(speeds.vyMetersPerSecond) < 0.01;
        if (stationary && isFacingTarget.getAsBoolean()) {
            brake();
        }
        else {
            driveFacingOptionalTarget(speeds, target);
        }
    }

    public Command brakeC(){
        return run(()->brake());
    }

    public void brake() {
        lastBrakeTime = Timer.getFPGATimestamp();
        setControl(brake);
    }

    private Rotation2d offsetTargetAngle(Angle target){
        var adjAngle = Rotation2d.fromDegrees(target.in(Degrees));
        var offset = getState().Pose.getRotation().minus(getGlobalPoseEstimate().getRotation());
        return adjAngle.plus(offset);
    }

    // public boolean driving() {
    //     return lastTargetSpeeds.equals(kSpeedsZero);
    // }

    // public Trigger drivingT() {
    //     return new Trigger(driving);
    // }

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
        visionEstimator.resetPosition(getState().RawHeading, getState().ModulePositions, pose);
    }

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
        sotmDriveSpeedRatio.poll();

        sotmLinearAccel.poll();
        sotmLinearDecel.poll();
        rotationDebounceTime.poll();
        // brakeDebounceSeconds.poll();

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
        
        // SOTM limiter
        if (sotmDriveSpeedRatio.hasChanged(hash) || turnSpeedRatio.hasChanged(hash) || sotmLinearAccel.hasChanged(hash)
                || sotmLinearDecel.hasChanged(hash) || angularAccel.hasChanged(hash) || angularDecel.hasChanged(hash)) {
            kSOTMLimiter.linearTopSpeed = getSOTMDriveSpeed();
            kSOTMLimiter.angularTopSpeed = getTurnSpeed();
            kSOTMLimiter.linearAcceleration = sotmLinearAccel.get();
            kSOTMLimiter.linearDeceleration = sotmLinearDecel.get();
            kSOTMLimiter.angularAcceleration = angularAccel.get();
            kSOTMLimiter.angularDeceleration = angularDecel.get();
        }
    }

    private void log() {
        SmartDashboard.putNumber("1) Drivetrain/Target Angle Deg", targetRotation.in(Degrees));
        SmartDashboard.putBoolean("1) Drivetrain/Facing Target Angle", isFacingTarget.getAsBoolean());
        SmartDashboard.putBoolean("1) Drivetrain/Turning To Face Target", turningToFaceTarget.getAsBoolean());
        SmartDashboard.putBoolean("1) Drivetrain/Is Braking", isBraking.getAsBoolean());
        var state = getState();
        if (state != null && state.Pose != null) {
            Rotation2d targetAngle = Shotmap.getFieldRelTargetFacingAngle(getGlobalPoseEstimate(), FieldUtil.kHubTrl);
            Rotation2d rawAngle = FieldUtil.kHubTrl.minus(getGlobalPoseEstimate().getTranslation()).getAngle();

            SmartDashboard.putNumber("1) Drivetrain/DriveFacingHub/TargetAngleDeg", targetAngle.plus(Rotation2d.k180deg).getDegrees());
            SmartDashboard.putNumber("1) Drivetrain/DriveFacingHub/RotationDeg", getGlobalPoseEstimate().getRotation().getDegrees());
            SmartDashboard.putNumber("1) Drivetrain/DriveFacingHub/RawAngleDeg", rawAngle.plus(Rotation2d.k180deg).getDegrees());
            SmartDashboard.putNumber("1) Drivetrain/Dist from hub", Shotmap.distanceToHub(getGlobalPoseEstimate()).in(Meters));
        }
    }
}
