package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.util.OCXboxController;

public class Superstructure {
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

    public Command passiveSpindexC() {
        return run(()->spindexer.setVoltage(IndexerConstants.spindexSlowVoltage.get()), spindexer);
        // either(
        //         run(()->spindexer.setVoltage(IndexerConstants.spindexSlowVoltage.get()), spindexer),
        //         spindexer.setVoltageC(0),
        //         feeder.topSensorT().negate()).withName("Index");
    }

    public Command shootShotMapControllerC(Supplier<OCXboxController> controller) {
        return shootShotMapC(OCDrivetrain.controllerToChassisSpeeds(controller));
    }

    public Command shootShotMapC(Supplier<ChassisSpeeds> speeds) {
        return parallel(
            run(
                () -> {
                    Distance distance = Shotmap.distanceToHub(drivetrain.getGlobalPoseEstimate());
                    Shooter.State state = Shotmap.getState(distance);

                    shooter.setState(state);

                    SmartDashboard.putNumber("Shooter/Shot/Distance", distance.in(Meters));
                    SmartDashboard.putNumber("Shooter/Shot/Angle", state.getAngle().in(Degrees));
                    SmartDashboard.putNumber("Shooter/Shot/RPM", state.getVelocity().in(RPM));
                    fourBar.oscillateC();
                },
                shooter, fourBar
            ), 
            drivetrain.driveFacingHub(speeds),
            sequence(
                waitUntil(() -> drivetrain.facingHubT().getAsBoolean() && shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean()),
                feeder.feedC(),
                spindexer.spindexC()
            )
        ).withName("ShootShotMapLive");
    }
}
