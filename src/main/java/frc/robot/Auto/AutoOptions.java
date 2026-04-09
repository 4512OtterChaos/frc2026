package frc.robot.Auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.RobotConstants.*;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.util.FieldUtil;
import frc.robot.util.OCPathPlannerAuto;

public class AutoOptions {
    private final AutoChooser autoChooser = new AutoChooser();
    private final OCDrivetrain drivetrain;
    private final Intake intake;
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final FourBar fourBar;
    private final Feeder feeder;
    private final Superstructure superstructure;

    private boolean autosSetup = false;
    RobotConfig robotConfig = new RobotConfig(kRobotWeight, kMOI, kModuleConfig, FL, FR, BL, BR);
    PathConstraints constraints = new PathConstraints(
        MetersPerSecond.of(2), 
        MetersPerSecondPerSecond.of(3), 
        DegreesPerSecond.of(540), 
        DegreesPerSecondPerSecond.of(720), 
        Volts.of(12)
    );
    PathConstraints fastConstraints = new PathConstraints(
        MetersPerSecond.of(3.5), 
        MetersPerSecondPerSecond.of(4.5), 
        DegreesPerSecond.of(540), 
        DegreesPerSecondPerSecond.of(720), 
        Volts.of(12)
    );

    // List<PathPlannerPath> currentPaths = new ArrayList<>();
    // int pathNumber = 0;

    // List<Trigger> pathTriggers = new ArrayList<>();

    public AutoOptions(OCDrivetrain drivetrain, Intake intake, Shooter shooter, Spindexer spindexer,
                       FourBar fourBar, Feeder feeder, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.fourBar = fourBar;
        this.feeder = feeder;
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

    public void periodic() {
        if (!autosSetup && !DriverStation.getAlliance().isEmpty()) {
            addOptions();
            log();
            autosSetup = true;
        }
    }

    private void addNamedCommands() {
        new EventTrigger("Intake")
            .onTrue(intake.setIsIntakingC(true))
            .onFalse(intake.setIsIntakingC(false));
        new EventTrigger("Lower Fourbar")
            .onTrue(fourBar.extendC());
        // new EventTrigger("Shoot")
        //     .onTrue(superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).withTimeout(4));
        new EventTrigger("Error Correct")
            .onTrue(pathfindToPathEnd());

        // NamedCommands.registerCommand("Intake", intake.setVoltageInC().asProxy());
        NamedCommands.registerCommand("Shoot", superstructure.otterShootStationaryC(()-> new ChassisSpeeds())/*.until(superstructure.hopperEmptyT) */.withTimeout(4).finallyDo(()->{shooter.setIdle();feeder.setVelocity(RPM.of(0));spindexer.setVoltage(0);}));
        NamedCommands.registerCommand("Shoot Forever", superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).finallyDo(()->{shooter.setIdle();feeder.setVelocity(RPM.of(0));spindexer.setVoltage(0);}));
        // NamedCommands.registerCommand("Lower Fourbar", fourBar.extendC().asProxy());
        NamedCommands.registerCommand("Error Correct Command", none());//pathfindToPathEnd());
    }

    public void addOptions() {
        autoChooser.addCmd("Shoot Preloads", 
            ()->sequence(
                runOnce(()-> drivetrain.resetPose(new Pose2d(Meters.of(4.5), FieldUtil.kFieldWidth.div(2), Rotation2d.k180deg)), drivetrain),
                drivetrain.driveC(()-> new ChassisSpeeds(-1, 0, 0), false).withTimeout(2.25),
                drivetrain.driveC(()-> new ChassisSpeeds(0, 0, 0), false).withTimeout(.5),
                waitSeconds(5),
                superstructure.otterShootStationaryC(()->new ChassisSpeeds())
            )
        );
        autoChooser.addCmd("Left 3x Faster Loop Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top 3x Faster Loop Bump Cycles"));
        autoChooser.addCmd("Right 3x Faster Loop Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top 3x Faster Loop Bump Cycles", true));
        autoChooser.addCmd("Left 3x Faster Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top 3x Faster Bump Cycles"));
        autoChooser.addCmd("Right 3x Faster Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top 3x Faster Bump Cycles", true));
        autoChooser.addCmd("Left Faster Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top Faster Bump Cycles"));
        autoChooser.addCmd("Right Faster Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top Faster Bump Cycles", true));
        autoChooser.addCmd("Left Faster Faster Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top Faster Faster Bump Cycles"));
        autoChooser.addCmd("Right Faster Faster Bump Cycles", ()-> OCPathPlannerAuto.buildAuto("Top Faster Faster Bump Cycles", true));
        autoChooser.addCmd("Left Double Cycle", ()-> OCPathPlannerAuto.buildAuto("Top Double Cycle"));
        autoChooser.addCmd("Right Double Cycle", ()-> OCPathPlannerAuto.buildAuto("Top Double Cycle", true));
        autoChooser.addCmd("Middle Depot", ()-> OCPathPlannerAuto.buildAuto("Middle Depot"));
        autoChooser.addCmd("Slower Middle Depot", ()-> OCPathPlannerAuto.buildAuto("Slower Middle Depot"));
    }

    public Command pathfindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, constraints);
    }

    public Command pathfindToPathEnd() {        
        return defer(()-> {
            Optional<PathPlannerPath> optionalPath = OCPathPlannerAuto.currentOrLastPath;

            if (optionalPath.isEmpty()){
                return none();
            }
            else {
                PathPlannerPath path = optionalPath.get();
                var pathPoints = path.getAllPathPoints();
                var translation = pathPoints.get(pathPoints.size() - 1).position;
                var rotation = path.getGoalEndState().rotation();

                return pathfindToPose(new Pose2d(translation, rotation));
            }
        }, Set.of(drivetrain));
    }

    public Command getAuto() {
        return Optional.ofNullable(autoChooser.selectedCommand()).orElse(none());
    }

    public void log() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
