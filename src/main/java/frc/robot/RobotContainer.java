// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureViz;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final OCXboxController driver = new OCXboxController(0);

    private final OCDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Intake intake = new Intake();
    private final FourBar fourBar = new FourBar();
    private final Spindexer spindexer = new Spindexer();
    private final Feeder feeder = new Feeder();
    private final Flywheel flywheel = new Flywheel();
    private final Hood hood = new Hood();
    private final Climber climber = new Climber();

    private final Vision vision = new Vision();

    private final Superstructure superstructure = new Superstructure(drivetrain, intake, fourBar, spindexer, feeder, flywheel, hood, climber);
    private final SuperstructureViz superstructureViz = new SuperstructureViz(drivetrain, intake, fourBar, spindexer, feeder, flywheel, hood, climber);

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    public void configureDefaultCommands(){
        drivetrain.setDefaultCommand(drivetrain.drive(driver));
        spindexer.setDefaultCommand(superstructure.passiveSpindexC());
        feeder.setDefaultCommand(feeder.passiveIndexC());
        flywheel.setDefaultCommand(flywheel.setVelocityC(ShooterConstants.flywheelIdleVelocity));
        intake.setDefaultCommand(intake.setVoltageC(0));
        hood.setDefaultCommand(hood.setAngleC(Degrees.of(0)));
        // drivetrain.setDefaultCommand(drivetrain.faceHub());
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    private void configureBindings() {

        driver.back().onTrue(runOnce(()-> drivetrain.resetRotation(Rotation2d.kZero)));
        driver.rightTrigger().whileTrue(parallel(superstructure.shootShotMapC(()-> Shotmap.distanceToHub(drivetrain.getGlobalPoseEstimate(), FieldUtil.kHubTrl)), drivetrain.driveFacingHub(driver)));
        driver.leftTrigger().whileTrue(intake.setVoltageInC());
        driver.a().whileTrue(fourBar.setMinAngleC());
        driver.y().whileTrue(fourBar.setMaxAngleC());
        driver.povUp().whileTrue(climber.setMaxHeightC());
        driver.povDown().whileTrue(climber.setMinHeightC());

        drivetrain.registerTelemetry(logger::telemeterize);
    
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    public void periodic() {
        vision.periodic();
        
        double phoenixTimeOffset = Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
        var swerveState = drivetrain.getState();
        vision.update(
            drivetrain.visionEstimator,
            swerveState.Pose.getRotation(),
            RadiansPerSecond.of(swerveState.Speeds.omegaRadiansPerSecond),
            swerveState.Timestamp + phoenixTimeOffset
        );
    }

    public void simulationPeriodic() {
        vision.simulationPeriodic(drivetrain.getState().Pose);
    }

}