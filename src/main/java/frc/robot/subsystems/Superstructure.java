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
import frc.robot.util.FieldUtil;
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
    
    /**
     * @param speeds
     * @param targetChooser true for hub, false for setpoint
     * @return
     */
    public Command shootShotMapControllerC(Supplier<OCXboxController> controller, boolean targetChooser) {
        return shootShotMapC(OCDrivetrain.controllerToChassisSpeeds(controller), targetChooser);
    }

    /**
     * @param speeds
     * @param targetChooser true for hub, false for setpoint
     * @return
     */
    public Command shootShotMapC(Supplier<ChassisSpeeds> speeds, boolean targetChooser) {
        return parallel(
            run(
                () -> {
                    Distance distance = Shotmap.distanceToTarget(drivetrain.getGlobalPoseEstimate(), targetChooser ? FieldUtil.kHubTrl : drivetrain.getGlobalPoseEstimate().nearest(FieldUtil.kSetpoints).getTranslation());
                    Shooter.State state = Shotmap.getState(distance);

                    shooter.setState(state);
                },
                shooter
            ), 
            targetChooser ? drivetrain.driveFacingHub(speeds) : drivetrain.driveFacingSetpoint(speeds), // TODO: use drivefacingHubController() instead?
            sequence(
                waitUntil(() -> shooter.upToSpeedT().getAsBoolean() && shooter.atAngleT().getAsBoolean()),
                feeder.feedC(),
                spindexer.spindexC()
            ), 
            fourBar.oscillateC()
        ).withName("ShootShotMap");
    }
}
