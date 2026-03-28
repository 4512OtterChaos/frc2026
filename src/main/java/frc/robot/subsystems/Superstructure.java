package frc.robot.subsystems;

import java.util.Optional; 
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds; 
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

public class Superstructure extends SubsystemBase{
    private OCDrivetrain drivetrain;
    private Intake intake;
    private FourBar fourBar;
    private Spindexer spindexer;
    private Feeder feeder;
    private Shooter shooter;
    private Climber climber;

    private boolean wasFeeding = false;
    private boolean doneShooting = false;
    public final Trigger doneShootingT = new Trigger(()->doneShooting);

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
     * 
     * @return A command that retracts the fourBar while running the intake
     */
    public Command fourbarRetractC() {
        return parallel(
            fourBar.retractC(),
            intake.setVoltageInC()
        );
    }
    
    /**
     * @param controller Controller to get strafe input from
     * @return A command sequence that turns to shoot into the hub, setting the drivetrain rotation, intake, fourbar, spindexer, feeder and shooter.
     */
    public Command otterShootStationaryControllerC(OCXboxController controller) {
        return otterShootStationaryC(OCDrivetrain.controllerToChassisSpeeds(controller));
    }
    
    /**
     * @param controller Controller to get strafe input from
     * @return
     */
    public Command otterShootOnTheSwimControllerC(OCXboxController controller) {
        return otterShootOnTheSwimC(OCDrivetrain.controllerToChassisSpeeds(controller));
    }
    
    /**
     * @param controller Controller to get strafe input from
     * @param target
     * @return
     */
    public Command otterShootControllerC(OCXboxController controller, Supplier<Optional<Translation2d>> target) {
        return otterShootStationaryC(OCDrivetrain.controllerToChassisSpeeds(controller), target);
    }
    
    /**
     * @param controller Controller to get strafe input from
     * @param target
     * @return
     */
    public Command otterShootOnTheSwimControllerC(OCXboxController controller, Supplier<Optional<Translation2d>> target) {
        return otterShootOnTheSwimC(OCDrivetrain.controllerToChassisSpeeds(controller), target);
    }

    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootStationaryC(Supplier<ChassisSpeeds> speeds) {
        return otterShootStationaryC(speeds, drivetrain.getTarget());
    } 

    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootOnTheSwimC(Supplier<ChassisSpeeds> speeds) {
        return otterShootOnTheSwimC(speeds, drivetrain.getTarget());
    } 

    /**
     * @param speeds Field relative chassis speeds of the gosh diddily dang robot
     * @param target the retail store with the dog with the red bullseye on his face as the mascot yk whayt im talking about. Target Corporation began as the Dayton Dry Goods Company, founded by George D. Dayton in 1902 in Minneapolis. The first Target discount store opened in Roseville, Minnesota, on May 1, 1962, aiming to offer high-quality goods at low prices. It grew into a national retailer, becoming the Dayton-Hudson Corporation in 1969 before renaming to Target Corporation in 2000. 
     * @return
     */
    public Command otterShootStationaryC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target) {
        Trigger hasTarget = new Trigger(()-> target.get().isEmpty()).negate();
        return sequence(
            parallel(
                shootStationaryBaseC(speeds, target),
                autoIndexForShooting(hasTarget)
            )
        ).finallyDo(()-> setDoneShooting(hasTarget)).withName("Otter Shoot Stationary");
    }

    /**
     * @param speeds Field relative chassis speeds
     * @param target
     * @return
     */
    public Command otterShootOnTheSwimC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target) {
        Trigger hasTarget = new Trigger(()-> target.get().isEmpty()).negate();
        return sequence(
            parallel(
                shootOnTheSwimBaseC(speeds, target, hasTarget),
                autoIndexForShooting(hasTarget)
            )
        ).finallyDo(()-> setDoneShooting(hasTarget)).withName("Otter Shoot on the Swim");
    }

    public Command shootStationaryBaseC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target){
        Trigger hasTarget = new Trigger(()-> target.get().isEmpty()).negate();
        return Commands.run(
            () -> {
                if (hasTarget.negate().getAsBoolean()) {
                    shooter.setIdleC();
                }
                else {
                    Distance distance = Shotmap.distanceToTarget(drivetrain.getGlobalPoseEstimate(), target.get().get());
                    Shooter.State state = Shotmap.getState(distance);

                    shooter.setState(state);
                }
                drivetrain.driveFacingOptionalTarget(speeds, target);
            },
            drivetrain, shooter
        ).withName("shootStationaryBaseC()");
    }

    public Command shootOnTheSwimBaseC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target, Trigger hasTarget){
        return Commands.run(
            () -> {
                if (hasTarget.negate().getAsBoolean()) {
                    shooter.setIdleC();
                }
                else{
                    var currSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
                    var vel = new Translation2d(currSpeeds.vxMetersPerSecond, currSpeeds.vyMetersPerSecond);
                    Translation2d hub = FieldUtil.kHubTrl;
                    Translation2d targetTrl = target.get().get();
                    Shooter.State state = null;

                    // perform TOF recursion for several iterations to account for the change in target position as the robot moves during the shot
                    for (int i = 0; i < 5; i++) {
                        Distance distance = Shotmap.distanceToTarget(drivetrain.getGlobalPoseEstimate(), targetTrl);
                        state = Shotmap.getState(distance);
                        targetTrl = hub.minus(vel.times(state.getTof().times(1).in(Seconds))); //TODO Tune compensation percentage?
                    }

                    shooter.setState(state);
                }
                drivetrain.driveFacingOptionalTargetSlowBrake(speeds.get(), target);
            },
            drivetrain, shooter
        ).withName("shootOnTheSwimBaseC()");
    }
    
    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootEndControllerC(OCXboxController controller) {
        return either(
            otterShootEndC(OCDrivetrain.controllerToChassisSpeeds(controller)),
            otterShootEndC(()-> new ChassisSpeeds()), 
            RobotModeTriggers.autonomous().negate()
        );
    }

    /**
     * @param speeds Field relative chassis speeds
     * @return
     */
    public Command otterShootEndC(Supplier<ChassisSpeeds> speeds) {
        return otterShootStationaryC(speeds, drivetrain.getTarget());
    } 

    /**
     * @param speeds Field relative chassis speeds 
     * @param target 
     * @return
     */
    public Command otterShootEndC(Supplier<ChassisSpeeds> speeds, Supplier<Optional<Translation2d>> target) {
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
            drivetrain.driveFacingOptionalTargetC(speeds, target),
            feeder.feedC()
        ).until(hasTarget.negate()).withTimeout(0.2).finallyDo(()->resetDoneShooting()).withName("Otter Shoot End");
    }

    public Command autoIndexForShooting(Trigger hasTarget){
        return repeatingSequence(
            parallel(
                waitUntil(hasTarget.debounce(0.2)),
                waitSeconds(0.7).until(()-> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean() && drivetrain.facingTargetT().getAsBoolean())
            ),
            runOnce(()->wasFeeding = true),
            parallel(
                fourBar.setReadyToOscillateC(true),
                indexC().asProxy()
            ).until(hasTarget.negate()).finallyDo(()-> fourBar.setReadyToOscillate(false))
        );
    }

    private void setDoneShooting(Trigger hasTarget){
        if (wasFeeding && hasTarget.getAsBoolean()) {
            doneShooting = true;
            wasFeeding = false;
        }
    }

    private void resetDoneShooting(){
        doneShooting = false;
    }

    // public static class ShootOnTheSwim {
    //     public static Pair<Shooter.State, Angle> getTargets(Pose2d robotPose, ChassisSpeeds speed){
    //         double latency = SOTMLatency.in(Seconds);
            
    //         Translation2d futurePos = robotPose.getTranslation().plus(
    //             new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond).times(latency)
    //         ).plus(RobotConstants.kShooterTranslation);

    //         Translation2d hubGoalLocation = FieldUtil.kHubTrl;
            
    //         Translation2d targetVec = hubGoalLocation.minus(futurePos);
    //         Distance dist = Meters.of(targetVec.getNorm());
    //         Translation2d targetDirection = targetVec.div(dist.in(Meters));

    //         Shooter.State baseline = Shotmap.getState(dist);
    //         double baselineVelocity = dist.in(Meters) / baseline.getTof().in(Seconds);

    //         Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    //         Translation2d robotVelVec = new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    //         Translation2d shotVelocity = targetVelocity.minus(robotVelVec).times(-1);

    //         Angle chassisAngle = Degrees.of(shotVelocity.getAngle().getDegrees());
    //         double requiredVelocity = shotVelocity.getNorm();

    //         double effectiveDistance = Shotmap.horizontalVelocityToEffectiveDistance(requiredVelocity);
    //         Shooter.State effectiveState = Shotmap.getState(Meters.of(effectiveDistance));

    //         return new Pair<Shooter.State,Angle>(effectiveState, chassisAngle);
    //     }
    // }   
}