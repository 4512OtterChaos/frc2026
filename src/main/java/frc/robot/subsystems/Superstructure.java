package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Shotmap;

public class Superstructure {
    private OCDrivetrain drivetrain;
    private Intake intake;
    private FourBar fourBar;
    private Spindexer spindexer;
    private Feeder feeder;
    private Flywheel flywheel;
    private Hood hood;

    public Superstructure(OCDrivetrain drivetrain, Intake intake, FourBar fourBar, Spindexer spindexer, Feeder feeder, Flywheel flywheel, Hood hood){
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.fourBar = fourBar;
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = flywheel;
        this.hood = hood;
    }

    public Command passiveSpindexC(){
        return either(
            spindexer.setVoltageC(IndexerConstants.kSpindexSlowVoltage),
            spindexer.setVoltageC(0),
            feeder.topSensorT().negate()
        ).withName("Index");
    }

    public Command shootShotMapC(Distance distance){
        return parallel(
            drivetrain.faceHub(),
            sequence(
                parallel(
                    waitUntil(drivetrain.facingHubT()),
                    flywheel.setVelocityC(Shotmap.getVelocity(distance)),
                    hood.setAngleC(Shotmap.getAngle(distance))
                ),
                parallel(
                    spindexer.spindexC(),
                    feeder.feedC()
                )
            )
        );
    }
}
