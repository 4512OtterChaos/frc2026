package frc.robot.Auto;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.util.RobotConstants.*;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.util.FieldUtil;

public class AutoOptions {
    private final AutoChooser autoChooser = new AutoChooser();
    private final OCDrivetrain drivetrain;
    private final Intake intake;
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final FourBar fourBar;
    private final Feeder feeder;
    private final Climber climber;
    private final Superstructure superstructure;

    private boolean autosSetup = false;
    RobotConfig robotConfig = new RobotConfig(kRobotWeight, kMOI, kModuleConfig, FL, FR, BL, BR);

    public AutoOptions(OCDrivetrain drivetrain, Intake intake, Shooter shooter, Spindexer spindexer,
                       FourBar fourBar, Climber climber, Feeder feeder, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.fourBar = fourBar;
        this.feeder = feeder;
        this.climber = climber;
        this.superstructure = superstructure;

        AutoBuilder.configure(
        () -> drivetrain.getGlobalPoseEstimate(),
        (pose) -> drivetrain.resetPose(pose),
        () -> drivetrain.getState().Speeds,
        (chassisSpeeds) -> drivetrain.driveAutos(chassisSpeeds),
        AutoConstants.kPathConfig,
        robotConfig,
        () -> false,
        drivetrain);

        addNamedCommands();
    }

    private void addNamedCommands() {
        NamedCommands.registerCommand("Intake", intake.setVoltageInC());
        NamedCommands.registerCommand("Shoot", superstructure.otterShootC(()-> new ChassisSpeeds()).withTimeout(3));
        NamedCommands.registerCommand("Wait", Commands.waitSeconds(1));
        // NamedCommands.registerCommand("Climber Up", climber.setMaxHeightC()); 
        // NamedCommands.registerCommand("Climber Down", climber.setMinHeightC());
        NamedCommands.registerCommand("Lower Fourbar", Commands.none() /*fourBar.setCurrentOutC().finallyDo(()->fourBar.setCurrent(Amps.of(0)))*/);
    }

    public void periodic() {
        if (!autosSetup && !DriverStation.getAlliance().isEmpty()) {
            addOptions();
            log();
            autosSetup = true;
        }
    }

    public void addOptions() {
        autoChooser.addCmd("New and super cool", 
            ()->sequence(
                runOnce(()->drivetrain.resetPose(new Pose2d(Meters.of(4.5), FieldUtil.kFieldWidth.div(2), Rotation2d.k180deg)), drivetrain),
                drivetrain.driveC(new ChassisSpeeds(-1, 0, 0)).withTimeout(2.25),
                drivetrain.driveC(new ChassisSpeeds(0, 0, 0)).withTimeout(.5),
                waitSeconds(5),
                superstructure.otterShootC(()->new ChassisSpeeds())
        ));
        autoChooser.addCmd("Right Double Cycle", ()->sequence(
            fourBar.setCurrentOutC().withTimeout(1),
            AutoBuilder.buildAuto("Bottom Double Cycle")
        ));
        autoChooser.addCmd("Left Double Cycle", ()->sequence(
            fourBar.setCurrentOutC().withTimeout(1),
            AutoBuilder.buildAuto("Top Double Cycle")
        ));

        //Individual autos
        autoChooser.addCmd("Right Shoot Climb", ()-> AutoBuilder.buildAuto("Top Shoot Climb"));
        autoChooser.addCmd("Left Shoot Climb", ()-> AutoBuilder.buildAuto("Bottom Shoot Climb"));
        autoChooser.addCmd("Right Depot Climb", ()-> AutoBuilder.buildAuto("Top Depot Climb"));
        // autoChooser.addCmd("Right Double Cycle", ()-> AutoBuilder.buildAuto("Top Double Cycle"));
        autoChooser.addCmd("Left Outpost", ()-> AutoBuilder.buildAuto("Bottom Outpost"));
        autoChooser.addCmd("Right Outpost", ()-> AutoBuilder.buildAuto("Top Outpost"));

        //Combined autos
        autoChooser.addCmd("Right Combo", ()-> AutoBuilder.buildAuto("Top Combo")); // TODO: climber is turned off btw
        autoChooser.addCmd("Left Combo", ()-> AutoBuilder.buildAuto("Bottom Combo")); // TODO: climber is turned off btw


    }

    public Command getAuto() {
        return Optional.ofNullable(autoChooser.selectedCommand()).orElse(none());
    }

    public void log() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
