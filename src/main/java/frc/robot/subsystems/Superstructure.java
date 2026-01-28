package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Shotmap;

public class Superstructure {
    private OCDrivetrain drivetrain;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Hood hood;

    public Superstructure(OCDrivetrain drivetrain, Intake intake, Indexer indexer, Flywheel flywheel, Hood hood){
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.indexer = indexer;
        this.flywheel = flywheel;
        this.hood = hood;
    }


    public Command shootShotMapC(Distance distance){
        return parallel(
            drivetrain.faceHub(),
            sequence(
                parallel(
                    waitUntil(drivetrain.facingHubT()),
                    flywheel.setVelocityC(Shotmap.getVelocity(distance)),
                    hood.setAngleC(Shotmap.getAngle(distance))
                )//,
                //indexer.index();
            )
        );
    }
}
