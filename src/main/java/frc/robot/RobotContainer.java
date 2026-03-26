// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Auto.AutoOptions;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureViz;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.FuelPhysicsSim;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.OCXboxController;
import frc.robot.util.RobotConstants;
import frc.robot.util.TunableNumber;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final OCXboxController driver = new OCXboxController(0);
    // private final OCXboxController operator = new OCXboxController(1);

    private final OCDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Intake intake = new Intake();
    private final FourBar fourBar = new FourBar();
    private final Spindexer spindexer = new Spindexer();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    // private final Climber climber = new Climber(); //TODO: Re-enable
    private final Vision vision = new Vision();

    private final Superstructure superstructure = new Superstructure(drivetrain, intake, fourBar, spindexer, feeder, shooter, null); // TODO: turn off climber when testing
    private final SuperstructureViz superstructureViz = new SuperstructureViz(drivetrain, intake, fourBar, spindexer, feeder, shooter, null);

    private final AutoOptions autos = new AutoOptions(drivetrain, intake, shooter, spindexer, fourBar, null, feeder, superstructure);

    TunableNumber feederVoltage = new TunableNumber("test/feederVoltage", 4);
    TunableNumber flywheelVelocity = new TunableNumber("test/flywheelVelocity", 1000);
    TunableNumber hoodAngle = new TunableNumber("test/hoodAngle", 15);

    public RobotContainer() {
        configureDefaultCommands();
        configureGeneralBindings();
        configureDriverBindings();
        configureOperatorBindings();

        DataLogManager.start();
    }

    public void configureDefaultCommands() {
        intake.setDefaultCommand(intake.setVoltageC(0));
        fourBar.setDefaultCommand(fourBar.setCurrentC(Amps.of(0)));
        spindexer.setDefaultCommand(spindexer.setVoltageC(0));
        feeder.setDefaultCommand(feeder.setVoltageC(0));
        shooter.setDefaultCommand(shooter.setIdleC());
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    private void configureGeneralBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);
        
        // Setup HubShiftUtil
        RobotModeTriggers.teleop().onTrue(runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.disabled().onTrue(runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    }


    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(drivetrain.driveC(driver, true));
        driver.back().onTrue(runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));
        driver.b().whileTrue(drivetrain.brakeC());
        driver.rightTrigger().whileTrue(superstructure.otterShootControllerC(driver, ()-> driver.leftTrigger().getAsBoolean()));
        driver.leftBumper().whileTrue(superstructure.otterShootOnTheSwimControllerC(driver));
        driver.rightBumper().whileTrue(parallel(
        // shooter.setFlywheelVoltage(
            run(()-> shooter.setState(Degrees.of(5), RPM.of(2200))), 
            sequence(
                waitSeconds(0.6),
                superstructure.indexC()
            )
        ));

        driver.leftTrigger().whileTrue(intake.setVoltageInC());
        driver.x().whileTrue(intake.setVoltageOutC());
        driver.povLeft().whileTrue(
            parallel(
                spindexer.reverseC(),
                feeder.reverseC()
        ));

        driver.y().whileTrue(fourBar.retractCurrentC().withTimeout(1.25)); 
        driver.a().whileTrue(fourBar.extendCurrentC().withTimeout(1)); 

        
        // driver.povUp().whileTrue(run(()->shooter.setState(Degrees.of(hoodAngle.get()), RPM.of(flywheelVelocity.get())))); //Testing command

        // driver.povUp().whileTrue(climber.setMaxHeightC()); //TODO: Re-enable
        // driver.povDown().whileTrue(climber.setMinHeightC()); //TODO: Re-enable
    }

    private void configureOperatorBindings() {
        // operator.y().onTrue(run(() -> Shotmap.setPresetState(Shotmap.nearH-p.backCorner)));

    }

    public Command getAutonomousCommand() {
        return autos.getAuto();
    }

    public void periodic() {
        vision.periodic();
        autos.periodic();
        // shooter.in
        log();
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

    public void AutonomousInit() {
        // CommandScheduler.getInstance().schedule(autoOptions.getAuto());
        // autoOptions.log();
    }

    public void log(){
        // Publish match time
        SmartDashboard.putNumber("Match Dashboard/Match Time", DriverStation.getMatchTime());

        // Update from HubShiftUtil
        SmartDashboard.putString(
            "Match Dashboard/Shifts/Remaining Shift Time",
            String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
        SmartDashboard.putBoolean("Match Dashboard/Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
        SmartDashboard.putString(
            "Match Dashboard/Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
        SmartDashboard.putBoolean(
            "Match Dashboard/Shifts/Active First?",
            DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());
    }

    public void changeTunable() {
        feederVoltage.poll();
        hoodAngle.poll();
        flywheelVelocity.poll();
    }

    //----- Simulation
    FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");
    Translation3d launcherPos = new Translation3d();
    final double launchGapTimeSec = 1.0 / 5; // 5 bps
    double lastLaunchTime = Timer.getFPGATimestamp();

    public void simulationInit() {
        ballSim.enable();
        // ballSim.placeFieldBalls();  // spawns all the game pieces

        // tell it about your robot
        ballSim.configureRobot(RobotConstants.kRobotWidth.in(Meters), RobotConstants.kRobotLength.in(Meters), Units.inchesToMeters(6.25),
            () -> drivetrain.getState().Pose, () -> drivetrain.getState().Speeds);
    }

    public void simulationPeriodic() {
        vision.simulationPeriodic(drivetrain.getState().Pose);

        // rudimentary shooting simulation: spawn fuel when everything is spinning
        double now = Timer.getFPGATimestamp();
        double flywheelRPM = shooter.getFlywheelVelocity().in(RPM);
        if (spindexer.getVelocity().gt(RPM.of(1000)) &&
                feeder.getVelocity().gt(RPM.of(1000)) &&
                flywheelRPM > 1000 &&
                now > lastLaunchTime + launchGapTimeSec) {
            // fuel exit velocity and spin based on current flywheel velocity
            var exitVelSpin = shooter.getFuelExitVelSpin();
            // (we actually ignore this exit vel as we estimate it another way using the shot map)
            ballSim.launchBall(
                shooter.getFuelExitPose(drivetrain.getState().Pose).getTranslation(),
                Shotmap.getRelativeFuelVels(shooter.getHoodAngle(), shooter.getFlywheelVelocity()).rotateBy(new Rotation3d(drivetrain.getState().Pose.getRotation().plus(Rotation2d.k180deg))),
                exitVelSpin.getSecond().in(RPM));
            lastLaunchTime = now;
        }
        ballSim.tick();  // runs physics, publishes ball positions to NT
    }
}
/*
 * TODO:
 * drivetrain current limits
 * tune autos (do they do everything right?)
 * shooter turn off delay
 * consider fourbar raising a lil while not intaking
 * ________________________________________________________________________________________________________________________
 * 
 * NOLAN'S TODO:
 * feeder pid, faster feeder
 * shooter kP
 * shooter fuel counter from current sensing. Debounce last shot time timeout for auto detecting empty hopper
 * current sensing fourbar
 *     detect stall by sensing when velocity = 0 and current > constant
 */