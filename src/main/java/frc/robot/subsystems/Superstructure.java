package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.SOTMLatency;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;
import frc.robot.util.RobotConstants;

public class Superstructure extends SubsystemBase{
    private OCDrivetrain drivetrain;
    private Intake intake;
    private FourBar fourBar;
    private Spindexer spindexer;
    private Feeder feeder;
    private Shooter shooter;
    private Climber climber;

    public Superstructure(OCDrivetrain drivetrain, Intake intake, FourBar fourBar, Spindexer spindexer, Feeder feeder,
             Shooter shooter, Climber climber) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.fourBar = fourBar;
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.shooter = shooter;
        this.climber = climber;
    }

    // public Command passiveSpindexC() {
    //     return either(
    //             run(()->spindexer.setVoltage(IndexerConstants.spindexSlowVoltage.get()), spindexer),
    //             spindexer.setVoltageC(0),
    //             feeder.topSensorT().negate()).withName("Index");
    // }
    
    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootControllerC(Supplier<OCXboxController> controller) {
        return otterShootC(OCDrivetrain.controllerToChassisSpeeds(controller));
    }
    
    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootControllerC(Supplier<OCXboxController> controller, Supplier<Optional<Translation2d>> target) {
        return otterShootC(OCDrivetrain.controllerToChassisSpeeds(controller), target);
    }

    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootC(Supplier<ChassisSpeeds> speeds) {
        return otterShootC(speeds, ()-> {
            if (drivetrain.inTrenchZone().getAsBoolean()) {
                return Optional.empty();
            }
            if (drivetrain.inAllianceZone().getAsBoolean()) {
                return Optional.of(FieldUtil.kHubTrl);
            }
            return Optional.of(drivetrain.getGlobalPoseEstimate().nearest(FieldUtil.kSetpoints).getTranslation());
        });
    } 

    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target) {
        Trigger hasTarget = new Trigger(()-> target.get().isEmpty()).negate();
        return parallel(
            Commands.run(
                () -> {
                    if (hasTarget.negate().getAsBoolean()) {
                        shooter.setIdleC();
                    }
                    else {
                        Distance distance = Shotmap.distanceToTarget(drivetrain.getGlobalPoseEstimate(), target.get().get());
                        Shooter.State state = Shotmap.getState(distance);

                        shooter.setState(state);
                    }
                },
                shooter
            ),
            drivetrain.driveFacingOptionalTarget(speeds, target),
            sequence(
                // waitUntil(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean() && drivetrain.facingTargetT(target).getAsBoolean()),
                parallel(
                    waitUntil(hasTarget.debounce(0.7)),
                    waitSeconds(0.7).until(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean())
                ),
                parallel(
                    feeder.feedC(),
                    spindexer.spindexC()
                ).until(hasTarget.negate())
            ).repeatedly()//, 
            // fourBar.oscillateC()
        ).withName("Otter Shoot");
    }
    
    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootOnTheSwimControllerC(Supplier<OCXboxController> controller) {
        return otterShootOnTheSwimC(OCDrivetrain.controllerToChassisSpeeds(controller));
    }
    

    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootOnTheSwimC(Supplier<ChassisSpeeds> speeds) {
        return parallel(
            Commands.run(
                () -> {
                    Pair<Shooter.State, Angle> targets = ShootOnTheMove.getTargets(drivetrain.getGlobalPoseEstimate(), drivetrain.getState().Speeds);

                    shooter.setState(targets.getFirst());
                    drivetrain.driveFacingAngle(speeds, targets.getSecond());
                },
                drivetrain, shooter
            ),
            sequence(
                // waitUntil(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean() && drivetrain.facingTargetT(target).getAsBoolean()),
                parallel(
                    waitSeconds(0.7).until(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean())
                ),
                parallel(
                    feeder.feedC(),
                    spindexer.spindexC()
                )
            ).repeatedly()//, 
            // fourBar.oscillateC()
        ).withName("Otter Shoot");
    }

    public static class ShootOnTheMove {
        public static Pair<Shooter.State, Angle> getTargets(Pose2d robotPose, ChassisSpeeds speed){
            double latency = SOTMLatency.in(Seconds);
            
            Translation2d futurePos = robotPose.getTranslation().plus(
                new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond).times(latency)
            ).plus(RobotConstants.kShooterTranslation);

            Translation2d hubGoalLocation = FieldUtil.kHubTrl;
            
            Translation2d targetVec = hubGoalLocation.minus(futurePos);
            Distance dist = Meters.of(targetVec.getNorm());
            Translation2d targetDirection = targetVec.div(dist.in(Meters));

            Shooter.State baseline = Shotmap.getState(dist);
            double baselineVelocity = dist.in(Meters) / baseline.getTof().in(Seconds);

            Translation2d targetVelocity = targetDirection.times(baselineVelocity);

            Translation2d robotVelVec = new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
            Translation2d shotVelocity = targetVelocity.minus(robotVelVec).times(-1);

            // double idealHorizontalSpeed = MetersPerSecond.of(rpmToMps(Shotmap.getVelocity(dist).in(RPM))); // TODO: holy fix this

            // Translation2d robotVelVec = new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
            // Translation2d shotVec = targetVec.div(dist.in(Meters)).times(idealHorizontalSpeed).minus(robotVelVec);

            Angle chassisAngle = Degrees.of(shotVelocity.getAngle().getDegrees());
            double requiredVelocity = shotVelocity.getNorm();

            // LinearVelocity totalExitVelocity = MetersPerSecond.of(15);

            // double ratio = Math.min(requiredVelocity.in(RPM) / totalExitVelocity.in(MetersPerSecond), 1.0);
            // double newPitch = Math.acos(ratio);

            double effectiveDistance = Shotmap.horizontalVelocityToEffectiveDistance(requiredVelocity);
            Shooter.State effectiveState = Shotmap.getState(Meters.of(effectiveDistance));

            return new Pair<Shooter.State,Angle>(effectiveState, chassisAngle);
        }

        // private final double WHEEL_RADIUS_METERS =0.03; //TODO: get the shooter flywheel radius
        // private final double SHOOTER_EFFICIENCY = 0.85; // TODO: tune

        // private double calcRPM(double totalExitVelocityMps) {
        //     double wheelAngularVelocity =
        //         totalExitVelocityMps / (WHEEL_RADIUS_METERS * SHOOTER_EFFICIENCY);

        //     return wheelAngularVelocity * (60.0 / (2.0 * Math.PI));
        // }

        // private LinearVelocity rpmToMps(AngularVelocity rpm) {
        //     return MetersPerSecond.of(67); // TODO: actually make this
        // }
    }   
}