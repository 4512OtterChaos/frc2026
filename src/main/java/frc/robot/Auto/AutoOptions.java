package frc.robot.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.util.RobotConstants.*;

import java.security.PrivateKey;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Shotmap;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCXboxController;

public class AutoOptions {
        public SendableChooser<Command> autoOptions = new SendableChooser<Command>();
        private OCDrivetrain drivetrain;
        private Intake intake;
        private Hood hood;
        private Flywheel flywheel;
        private Spindexer spindexer;
        private FourBar fourBar;
        private Feeder feeder;
        private Climber climber;
        private OCXboxController driver;
        private Superstructure superstructure;

        private boolean autosSetup = false;
        RobotConfig robotConfig = new RobotConfig(kRobotWeight, kMOI, kModuleConfig, FL, FR, BL, BR);

        public AutoOptions(OCDrivetrain drivetrain, Intake intake, Hood hood, Flywheel flywheel, Spindexer spindexer,
                        FourBar fourBar, Climber climber, Feeder feeder, Superstructure superstructure) {
                this.drivetrain = drivetrain;
                this.intake = intake;
                this.hood = hood;
                this.flywheel = flywheel;
                this.spindexer = spindexer;
                this.fourBar = fourBar;
                this.feeder = feeder;
                this.climber = climber;
                this.superstructure = superstructure;

                // AutoBuilder.configure(
                // () -> drivetrain.getGlobalPoseEstimate(),
                // (pose) -> drivetrain.resetPose(pose),
                // () -> drivetrain.getState().Speeds,
                // (chassisSpeeds) -> drivetrain.drive(chassisSpeeds),
                // AutoConstants.kPathConfig,
                // robotConfig,
                // () -> drivetrain.driveMirror(),
                // drivetrain, intake, hood, flywheel, spindexer, fourBar, feeder, climber);

                addAutoMethods();
        }

        public Command shootC() {
                return Commands.parallel(
                                superstructure.shootShotMapC(
                                                () -> Shotmap.distanceToHub(drivetrain.getGlobalPoseEstimate(),
                                                                FieldUtil.kHubTrl)),
                                drivetrain.driveFacingHub(driver));
        }

        private void addAutoMethods() {
                NamedCommands.registerCommand("Intake", intake.setVoltageInC());
                NamedCommands.registerCommand("Shoot", shootC());
        }

        public void periodic() {
                if (!autosSetup && !DriverStation.getAlliance().isEmpty()) {
                        autoOptions.setDefaultOption("none", drivetrain.resetInitialOdomC());
                        addTopShootClimbOption();
                        addBottomShootClimbOption();
                        addTopDepotClimbOption();
                        log();
                        autosSetup = true;
                }
        }

        public void addTopShootClimbOption() {
                autoOptions.addOption("1 - Shoot",
                                AutoBuilder.buildAuto("Shoot"));
                autoOptions.addOption("1 - ClimberUp",
                                AutoBuilder.buildAuto("ClimberUp"));
                autoOptions.addOption("1 - ClimberDown",
                                AutoBuilder.buildAuto("ClimberDown"));
        }

        public void addBottomShootClimbOption() {
                autoOptions.addOption("2 - ClimberUp",
                                AutoBuilder.buildAuto("ClimberUp"));
                autoOptions.addOption("2 - ClimberDown",
                                AutoBuilder.buildAuto("ClimberDown"));
                autoOptions.addOption("2 - Shoot",
                                AutoBuilder.buildAuto("Shoot"));
        }

        public void addTopDepotClimbOption() {
                autoOptions.addOption("3 - Intake",
                                AutoBuilder.buildAuto("Intake"));
                autoOptions.addOption("3 - Shoot",
                                AutoBuilder.buildAuto("Shoot"));
                autoOptions.addOption("3 - ClimberUp",
                                AutoBuilder.buildAuto("ClimberUp"));
                autoOptions.addOption("3 - ClimberDown",
                                AutoBuilder.buildAuto("ClimberDown"));
        }

        public Command getAuto() {
                return Optional.ofNullable(autoOptions.getSelected()).orElse(none());
        }

        public void log() {
                SmartDashboard.putData("Auto Chooser", autoOptions);
        }
}
