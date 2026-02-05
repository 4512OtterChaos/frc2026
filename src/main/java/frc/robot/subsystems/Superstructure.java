package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            spindexer.setVoltageC(IndexerConstants.spindexSlowVoltage.get()),
            spindexer.setVoltageC(0),
            feeder.topSensorT().negate()
        ).withName("Index");
    }

    public Command shootShotMapC(Supplier<Distance> distanceSup) {
        Command liveSetpoints = run(() -> {
            var d = distanceSup.get();         
            var state = Shotmap.getState(d);   

            hood.setAngle(state.getAngle());   
            flywheel.setVelocity(state.getVelocity());

            // Debug proof (optional but super helpful)
            SmartDashboard.putNumber("Shot/Distance", d.in(Meters));
            SmartDashboard.putNumber("Shot/CMD Angle", state.getAngle().in(Degrees));
            SmartDashboard.putNumber("Shot/CMD RPM", state.getVelocity().in(RPM));
        });

        Command shoot = parallel(spindexer.spindexC(), feeder.feedC());

        return parallel(
            liveSetpoints, 
            sequence(
                waitUntil(() -> drivetrain.facingHubT().getAsBoolean() && flywheel.upToSpeed()),
                flywheel.setVelocityC(RPM.of(90))
            )
        ).withName("ShootShotMapLive");
    }


    
}
