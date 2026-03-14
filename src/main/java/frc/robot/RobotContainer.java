// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
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
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodMinAngle;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureViz;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.OCXboxController;
import frc.robot.util.TunableNumber;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed

    private final OCXboxController driver = new OCXboxController(0);
    private final OCXboxController operator = new OCXboxController(1);

    private final OCDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Intake intake = new Intake();
    private final FourBar fourBar = new FourBar(); //TODO: Re-enable
    private final Spindexer spindexer = new Spindexer();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber(); //TODO: Re-enable
    private final Vision vision = new Vision();

    private final Superstructure superstructure = new Superstructure(drivetrain, intake, fourBar, spindexer, feeder, shooter, null); // TODO: turn off climber/FB when testing
    private final SuperstructureViz superstructureViz = new SuperstructureViz(drivetrain, intake, fourBar, spindexer, feeder, shooter, null);

    private final AutoOptions autos = new AutoOptions(drivetrain, intake, shooter, spindexer, fourBar, climber, feeder, superstructure);

    TunableNumber feederVoltage = new TunableNumber("test/feederVoltage", 4);
    TunableNumber flywheelVelocity = new TunableNumber("test/flywheelVelocity", 1000);
    TunableNumber hoodAngle = new TunableNumber("test/hoodAngle", 15);

    private boolean driverShoot = true;

    public RobotContainer() {
        configureDefaultCommands();
        configureGeneralBindings();
        configureDriverBindings();
        configureOperatorBindings();
    }

    public void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.driveC(driver));
        spindexer.setDefaultCommand(spindexer.setVoltageC(0));
        feeder.setDefaultCommand(feeder.setVoltageC(0));
        spindexer.setDefaultCommand(spindexer.setVoltageC(0));
        feeder.setDefaultCommand(feeder.setVoltageC(0));
        shooter.setDefaultCommand(shooter.setStateC(kHoodMinAngle, RPM.of(ShooterConstants.flywheelIdleRPM.get())));
        intake.setDefaultCommand(intake.setVoltageC(0));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    private void configureGeneralBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);
        
        // Setup HubShiftUtil
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    }


    private void configureDriverBindings() {
        driver.back().onTrue(runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));
        if (driverShoot) {
            driver.rightTrigger().whileTrue(superstructure.shootShotMapControllerC(() -> driver));
        } else {
            driver.rightTrigger().whileTrue(run(()-> shooter.setState(Shooter.State.operatorState)));
        }
        // driver.a().whileTrue(parallel(
        //     spindexer.spindexC(),
        //     feeder.feedC()
        // ));
        driver.leftTrigger().whileTrue(intake.setVoltageInC());
        driver.y().whileTrue(fourBar.setCurrentInC()); //TODO: Re-enable
        driver.a().whileTrue(fourBar.setCurrentOutC()); //TODO: Re-enable
        // driver.povUp().whileTrue(climber.setMaxHeightC()); //TODO: Re-enable
        // driver.povDown().whileTrue(climber.setMinHeightC()); //TODO: Re-enable
    }

    private void configureOperatorBindings() {
        Distance nearHub = Meters.of(234543); // TODO: tune all of these
        Distance trench = Meters.of(236);
        Distance frontOfTower = Meters.of(2236);
        Distance nextToTower = Meters.of(233565677);
        
        operator.y().whileTrue(run(() -> Shooter.State.setOperatorState(nearHub))); //TODO: Re-enable
        operator.b().whileTrue(run(() -> Shooter.State.setOperatorState(trench))); //TODO: Re-enable
        operator.a().whileTrue(run(() -> Shooter.State.setOperatorState(frontOfTower))); //TODO: Re-enable
        operator.x().whileTrue(run(() -> Shooter.State.setOperatorState(nextToTower))); //TODO: Re-enable

        operator.back().onTrue(runOnce(()-> driverShoot = true)); // Is this ok?
        operator.start().onTrue(runOnce(()-> driverShoot = false));

    }

    public Command getAutonomousCommand() {
        return autos.getAuto();
    }

    public void periodic() {
        Shotmap.periodic();
        vision.periodic();
        autos.periodic();
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

    public void simulationPeriodic() {
        vision.simulationPeriodic(drivetrain.getState().Pose); 
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
}
/*
 * TODO:
 * Shoot command intake agitation
 * Four Bar torque control
 * Shooter torque control
 * Fix Autonomous Routine to not crash
 */