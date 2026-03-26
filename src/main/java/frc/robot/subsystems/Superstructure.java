package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
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
import frc.robot.subsystems.Indexer.*;
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

    public Superstructure(OCDrivetrain drivetrain, Intake intake, FourBar fourBar, Spindexer spindexer, Feeder feeder, Shooter shooter, Climber climber) {
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

    public Command indexC() {
        return parallel(
            feeder.feedC(),
            sequence(
                waitSeconds(0.1),
                parallel(
                    spindexer.spindexC(),
                    intake.setVoltageInC().asProxy()
                )
            )
        );
    }
    
    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootControllerC(OCXboxController controller, BooleanSupplier isIntakePressed) {
        return otterShootC(OCDrivetrain.controllerToChassisSpeeds(controller), isIntakePressed);
    }
    
    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootControllerC(OCXboxController controller, Supplier<Optional<Translation2d>> target) {
        return otterShootC(OCDrivetrain.controllerToChassisSpeeds(controller), target, ()-> false);
    }

    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootC(Supplier<ChassisSpeeds> speeds, BooleanSupplier isIntakePressed) {
        return otterShootC(speeds, ()-> {
            if (drivetrain.inTrenchZone().getAsBoolean()) {
                return Optional.empty();
            }
            if (drivetrain.inAllianceZone().getAsBoolean()) {
                return Optional.of(FieldUtil.kHubTrl);
            }
            return Optional.of(drivetrain.getGlobalPoseEstimate().nearest(FieldUtil.kSetpoints).getTranslation());
        }, 
        isIntakePressed);
    } 

    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @param isIntakePressed is da intake pressed or is it not or is it like in the middle or somehting idk
     * @return
     */
    public Command otterShootC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target, BooleanSupplier isIntakePressed) {
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
                    waitSeconds(0.7).until(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean() && drivetrain.facingTargetT().getAsBoolean())
                ),
                parallel(
                    fourBar.oscillateC().onlyWhile(()-> !isIntakePressed.getAsBoolean()).repeatedly(),
                    indexC()
                ).until(hasTarget.negate())//.andThen(feeder.feedC()).withTimeout(kShooterTurnOffDelay) TODO: fix
            ).repeatedly()            
        ).withName("Otter Shoot");
    }
    
    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootOnTheSwimControllerC(OCXboxController controller) {
        return otterShootOnTheSwimC(OCDrivetrain.controllerToChassisSpeeds(controller));
    }
    

    // /**
    //  * @param speeds Field relative chassis speeds
    //  * @param target
    //  * @return
    //  */
    // public Command otterShootOnTheSwimC(Supplier<ChassisSpeeds> speeds) {
    //     return parallel(
    //         Commands.run(
    //             () -> {
    //                 Pair<Shooter.State, Angle> targets = ShootOnTheMove.getTargets(drivetrain.getGlobalPoseEstimate(), drivetrain.getState().Speeds);

    //                 shooter.setState(targets.getFirst());
    //                 drivetrain.driveFacingAngle(speeds, targets.getSecond());
    //             },
    //             drivetrain, shooter
    //         ),
    //         sequence(
    //             // waitUntil(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean() && drivetrain.facingTargetT(target).getAsBoolean()),
    //             parallel(
    //                 waitSeconds(0.7).until(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean())
    //             ),
    //             indexC()
    //         ).repeatedly()//, 
    //         // fourBar.oscillateC()
    //     ).withName("Otter Shoot");
    // }

    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootOnTheSwimC(Supplier<ChassisSpeeds> speeds) {
        return parallel(
            Commands.run(
                () -> {
                    var currSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
                    var vel = new Translation2d(currSpeeds.vxMetersPerSecond, currSpeeds.vyMetersPerSecond);
                    Translation2d hub = FieldUtil.kHubTrl;
                    Translation2d target = hub;
                    Shooter.State state = null;

                    // perform TOF recursion for several iterations to account for the change in target position as the robot moves during the shot
                    for (int i = 0; i < 5; i++) {
                        Distance distance = Shotmap.distanceToTarget(drivetrain.getGlobalPoseEstimate(), target);
                        state = Shotmap.getState(distance);
                        // TODO: magic 0.25 coefficient, is this just simulation?
                        target = hub.minus(vel.times(state.getTof().times(0.25).in(Seconds)));
                    }

                    shooter.setState(state);
                    drivetrain.driveFacingTargetSlowBrake(speeds.get(), target);
                },
                drivetrain, shooter
            ),
            sequence(
                // waitUntil(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean() && drivetrain.facingTargetT(target).getAsBoolean()),
                parallel(
                    waitSeconds(0.7).until(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean() && drivetrain.facingTargetT().getAsBoolean())
                ),
                indexC() //.andThen(feeder.feedC()).withTimeout(kShooterTurnOffDelay) TODO: fix
            ).repeatedly()//, 
            // fourBar.oscillateC()
        ).withName("Otter Shoot");
    }

    public Command fourbarRetractC() {
        return parallel(
            fourBar.retractC(),
            intake.setVoltageInC()
        );
    }

    public static class ShootOnTheSwim {
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

            Angle chassisAngle = Degrees.of(shotVelocity.getAngle().getDegrees());
            double requiredVelocity = shotVelocity.getNorm();

            double effectiveDistance = Shotmap.horizontalVelocityToEffectiveDistance(requiredVelocity);
            Shooter.State effectiveState = Shotmap.getState(Meters.of(effectiveDistance));

            return new Pair<Shooter.State,Angle>(effectiveState, chassisAngle);
        }
    }   
}