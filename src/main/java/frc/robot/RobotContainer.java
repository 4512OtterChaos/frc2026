// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Superstructure;
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

    private final Superstructure superstructure = new Superstructure(drivetrain, intake, fourBar, spindexer, feeder, flywheel, hood);

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    public void configureDefaultCommands(){
        drivetrain.setDefaultCommand(drivetrain.drive(driver));
        spindexer.setDefaultCommand(superstructure.passiveSpindexC());
        feeder.setDefaultCommand(feeder.passiveIndexC());
        flywheel.setDefaultCommand(flywheel.setVelocityC(ShooterConstants.flywheelIdleVelocity));
        hood.setDefaultCommand(hood.setMinAngleC());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    private void configureBindings() {

        // Reset the field-centric heading on left bumper press.
        driver.back().onTrue(runOnce(()-> drivetrain.resetRotation(Rotation2d.kZero)));

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
}