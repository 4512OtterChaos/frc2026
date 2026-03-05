// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Auto.AutoOptions;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureViz;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;
import frc.robot.util.TunableNumber;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed

    private final OCXboxController driver = new OCXboxController(0);

    private final OCDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Intake intake = new Intake();
    private final FourBar fourBar = new FourBar();
    private final Spindexer spindexer = new Spindexer();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Vision vision = new Vision();

    private final Superstructure superstructure = new Superstructure(drivetrain, intake, fourBar, spindexer, feeder, shooter, climber);
    private final SuperstructureViz superstructureViz = new SuperstructureViz(drivetrain, intake, fourBar, spindexer, feeder, shooter, climber);

    // // private final PathPlannerAuto pathPlannerAuto = new PathPlannerAuto("Top
    // // Depot Climb");
    // private final AutoOptions autoOptions = new AutoOptions(drivetrain, intake,
    // hood, flywheel, spindexer, fourBar,
    // climber, feeder, superstructure);

    TunableNumber feederVoltage = new TunableNumber("test/feederVoltage", 4);
    TunableNumber flywheelVelocity = new TunableNumber("test/flywheelVelocity", 1000);
    TunableNumber hoodAngle = new TunableNumber("test/hoodAngle", 15);

    public RobotContainer() {
        setupPathPlanner();
        configureDefaultCommands();
        configureBindings();
    }

    public void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.drive(driver));
        spindexer.setDefaultCommand(superstructure.passiveSpindexC());
        feeder.setDefaultCommand(feeder.passiveIndexC());
        shooter.setDefaultCommand(shooter.setStateC(kHoodMinAngle, RPM.of(ShooterConstants.flywheelIdleRPM.get())));
        intake.setDefaultCommand(intake.setVoltageC(0));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    private void configureBindings() {
        // driver.a().whileTrue(run(() -> flywheel.setVelocity(RPM.of(flywheelVelocity.get())), flywheel));
        // driver.b().whileTrue(run(() -> hood.setAngle(Degrees.of(hoodAngle.get())), hood));
        // driver.rightTrigger().whileTrue(run(() -> feeder.setVoltage(feederVoltage.get()), feeder))
        //         .onFalse(runOnce(() -> feeder.setVoltage(0), feeder));
        driver.back().onTrue(runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));
        driver.rightTrigger().whileTrue(superstructure.shootShotMapControllerC(()->driver));
        driver.leftTrigger().whileTrue(intake.setVoltageInC());
        driver.a().whileTrue(fourBar.setCurrentOutC());
        driver.y().whileTrue(fourBar.setCurrentInC());
        driver.povUp().whileTrue(climber.setMaxHeightC());
        driver.povDown().whileTrue(climber.setMinHeightC());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.none();// sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Finally idle for the rest of auton
        // drivetrain.applyRequest(() -> idle),

        // run(() -> CommandScheduler.getInstance().schedule(autoOptions.getAuto())),

        // drivetrain.getAutonomousCommand("Top Depot Climb"));
    }

    public void periodic() {
        Shotmap.periodic();
        vision.periodic();
        changeTunable();

        double phoenixTimeOffset = Timer.getFPGATimestamp() -
        Utils.getCurrentTimeSeconds();
        var swerveState = drivetrain.getState();
        vision.update(
        drivetrain.visionEstimator,
        swerveState.Pose.getRotation(),
        RadiansPerSecond.of(swerveState.Speeds.omegaRadiansPerSecond),
        swerveState.Timestamp + phoenixTimeOffset);
    }

    public void simulationPeriodic() {
        vision.simulationPeriodic(drivetrain.getState().Pose);

        vision.update(
                drivetrain.visionEstimator,
                drivetrain.getState().Pose.getRotation(),
                RadiansPerSecond.of(drivetrain.getState().Speeds.omegaRadiansPerSecond),
                drivetrain.getState().Timestamp + Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds());
    }

    public void autonomousInit() {
        // CommandScheduler.getInstance().schedule(autoOptions.getAuto());
        // autoOptions.log();
    }

    public void AutonomousInit() {
        autonomousInit();
    }

    public void setupPathPlanner() {
        // try {
        // RobotConfig config = RobotConfig.fromGUISettings();

        // AutoBuilder.configure(
        // () -> drivetrain.getGlobalPoseEstimate(),
        // drivetrain::resetOdometry,
        // () ->
        // drivetrain.getKinematics().toChassisSpeeds(drivetrain.getState().ModuleStates),
        // (speeds) -> {
        // drivetrain.setControl(drivetrain.getDriveRequest()
        // .withVelocityX(speeds.vxMetersPerSecond)
        // .withVelocityY(speeds.vyMetersPerSecond)
        // .withRotationalRate(speeds.omegaRadiansPerSecond));
        // },

        // new PPHolonomicDriveController(
        // new PIDConstants(5.0, 0.0, 0.0), // Translation
        // new PIDConstants(5.0, 0.0, 0.0) // Rotation
        // ),
        // config,

        // () -> DriverStation.getAlliance()
        // .map(a -> a == DriverStation.Alliance.Red)
        // .orElse(false),

        // drivetrain);

        // } catch (Exception e) {
        // e.printStackTrace();
        // }

        // CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    public void changeTunable() {
        feederVoltage.poll();
        hoodAngle.poll();
        flywheelVelocity.poll();
    }
}
/*
 * TODO:
 * Shoot command intake agitation
 * Four Bar torque control
 * Shooter torque control
 * Fix Autonomous Routine to not crash
 */