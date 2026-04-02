package frc.robot.Auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.RobotConstants.*;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        new EventTrigger("Intake")
            .onTrue(intake.setIsIntakingC(true))
            .onFalse(intake.setIsIntakingC(false));
        new EventTrigger("Lower Fourbar")
            .onTrue(fourBar.extendC());
        new EventTrigger("Shoot")
            .onTrue(superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).withTimeout(4));

        // NamedCommands.registerCommand("Intake", intake.setVoltageInC().asProxy());
        NamedCommands.registerCommand("Shoot", superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).until(superstructure.emptyHopperDetectionT).withTimeout(4).finallyDo(()->{shooter.setIdle();feeder.setVelocity(RPM.of(0));spindexer.setVoltage(0);}));
        // NamedCommands.registerCommand("Shoot", superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).until(shooter.emptyHopperT()).finallyDo(()->{shooter.setIdle();feeder.setVelocity(RPM.of(0));spindexer.setVoltage(0);})); // TODO: test
        NamedCommands.registerCommand("Shoot Forever", superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).finallyDo(()->{shooter.setIdle();feeder.setVelocity(RPM.of(0));spindexer.setVoltage(0);}));
        // NamedCommands.registerCommand("Lower Fourbar", fourBar.extendC().asProxy());
    }

    public void periodic() {
        if (!autosSetup && !DriverStation.getAlliance().isEmpty()) {
            addOptions();
            log();
            autosSetup = true;
        }
    }

    public void addOptions() {
        autoChooser.addCmd("Shoot Preloads", 
            ()->sequence(
                runOnce(()-> drivetrain.resetPose(new Pose2d(Meters.of(4.5), FieldUtil.kFieldWidth.div(2), Rotation2d.k180deg)), drivetrain),
                drivetrain.driveC(()-> new ChassisSpeeds(-1, 0, 0), false).withTimeout(2.25),
                drivetrain.driveC(()-> new ChassisSpeeds(0, 0, 0), false).withTimeout(.5),
                waitSeconds(5),
                superstructure.otterShootStationaryC(()->new ChassisSpeeds())
        ));
        autoChooser.addCmd("Left Bump Cycles", ()-> AutoBuilder.buildAuto("Top Bump Cycles"));
        autoChooser.addCmd("Right Bump Cycles", ()-> new PathPlannerAuto("Top Bump Cycles", true));
        autoChooser.addCmd("Faster Left Bump Cycles", ()-> AutoBuilder.buildAuto("Faster Top Bump Cycles"));
        autoChooser.addCmd("Faster Right Bump Cycles", ()-> new PathPlannerAuto("Faster Top Bump Cycles", true));
        autoChooser.addCmd("Faster Faster Left Bump Cycles", ()-> AutoBuilder.buildAuto("Faster Faster Top Bump Cycles"));
        autoChooser.addCmd("Faster Faster Right Bump Cycles", ()-> new PathPlannerAuto("Faster Faster Top Bump Cycles", true));
        autoChooser.addCmd("Left Double Cycle", ()-> AutoBuilder.buildAuto("Top Double Cycle"));
        autoChooser.addCmd("Right Double Cycle", ()-> new PathPlannerAuto("Top Double Cycle", true));
        autoChooser.addCmd("Middle Depot", ()-> AutoBuilder.buildAuto("Middle Depot"));
    }

    public Command getAuto() {
        return Optional.ofNullable(autoChooser.selectedCommand()).orElse(none());
    }

    public void log() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
