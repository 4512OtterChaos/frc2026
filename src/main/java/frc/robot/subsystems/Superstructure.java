package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

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
        return either(
                spindexer.setVoltageC(IndexerConstants.spindexSlowVoltage.get()),
                spindexer.setVoltageC(0),
                feeder.topSensorT().negate()).withName("Index");
    }

    public Command shootShotMapC(Supplier<Distance> distanceSup) {
        return run(
                () -> {
                    var distance = distanceSup.get();
                    var state = Shotmap.getState(distance);

                    shooter.setState(state.getAngle(), state.getVelocity());

                    SmartDashboard.putNumber("Shot/Distance", distance.in(Meters));
                    SmartDashboard.putNumber("Shot/Angle", state.getAngle().in(Degrees));
                    SmartDashboard.putNumber("Shot/RPM", state.getVelocity().in(RPM));
                },
                shooter);
    }

    public Command shootShotMapC() {
        Supplier<Distance> distanceSup = () -> Shotmap.distanceToHub(drivetrain.getGlobalPoseEstimate(),
                FieldUtil.kHubTrl);
        return shootShotMapC(distanceSup);
    }
}
