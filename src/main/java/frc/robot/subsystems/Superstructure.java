package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
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
}