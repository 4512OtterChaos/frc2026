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

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    PathConstraints constraints = new PathConstraints(
        MetersPerSecond.of(3.5), 
        MetersPerSecondPerSecond.of(4.5), 
        DegreesPerSecond.of(540), 
        DegreesPerSecondPerSecond.of(720), 
        Volts.of(12)
    );

    List<PathPlannerPath> currentPaths = new ArrayList<>();
    int pathNumber = 0;

    // List<Trigger> pathTriggers = new ArrayList<>();

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
        NamedCommands.registerCommand("Shoot", superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).until(superstructure.hopperEmptyT).withTimeout(4).finallyDo(()->{shooter.setIdle();feeder.setVelocity(RPM.of(0));spindexer.setVoltage(0);}));
        NamedCommands.registerCommand("Shoot Forever", superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).finallyDo(()->{shooter.setIdle();feeder.setVelocity(RPM.of(0));spindexer.setVoltage(0);}));
        // NamedCommands.registerCommand("Lower Fourbar", fourBar.extendC().asProxy());
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
        autoChooser.addCmd("Left Bump Cycles", buildAuto("Top Bump Cycles"));
        autoChooser.addCmd("Right Bump Cycles", buildAuto("Top Bump Cycles", true));
        autoChooser.addCmd("Faster Left Bump Cycles", buildAuto("Faster Top Bump Cycles"));
        autoChooser.addCmd("Faster Right Bump Cycles", buildAuto("Faster Top Bump Cycles", true));
        autoChooser.addCmd("Faster Faster Left Bump Cycles", buildAuto("Faster Faster Top Bump Cycles"));
        autoChooser.addCmd("Faster Faster Right Bump Cycles", buildAuto("Faster Faster Top Bump Cycles", true));
        autoChooser.addCmd("Left Double Cycle", buildAuto("Top Double Cycle"));
        autoChooser.addCmd("Right Double Cycle", buildAuto("Top Double Cycle", true));
        autoChooser.addCmd("Middle Depot", buildAuto("Middle Depot"));
    }

    private Supplier<Command> buildAuto(String name){
        return buildAuto(name, false);
    }

    private Supplier<Command> buildAuto(String name, boolean mirror){
        return ()->{
            PathPlannerAuto auto = new PathPlannerAuto(name, mirror);

            try {
                currentPaths = PathPlannerAuto.getPathGroupFromAutoFile(name);
                pathNumber = 0;
            } catch (IOException | ParseException e) {
                System.out.println("Failed to find auto paths!");
                e.printStackTrace();
            }

            // pathTriggers = new ArrayList<>();
            // for (int i = 0; i < currentPaths.size(); i++) {
            //     pathTriggers.add(auto.activePath(currentPaths.get(i).name));
            // }
            
            return auto;
        };
    }

    public Optional<PathPlannerPath> getCurrentPath(){
        if (currentPaths.size() <= pathNumber){
            return Optional.empty();
        }
        return Optional.of(currentPaths.get(pathNumber));
    }

    // public Optional<PathPlannerPath> getCurrentPath(){
    //     for(int i = 0; i < pathTriggers.size(); i++){
    //         if (pathTriggers.get(i).getAsBoolean()) {
    //             return Optional.of(currentPaths.get(i));
    //         }
    //     }
    //     for(int i = 0; i < pathTriggers.size(); i++){
    //         System.out.println(pathTriggers.get(i).getAsBoolean());
    //     }
    //     return Optional.empty();
    // }

    public Command pathfindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, constraints);
    }

    public Command pathfindToPathEnd() {
        Set<Subsystem> requirements = new HashSet<Subsystem>();
        requirements.add(drivetrain);
        
        return defer(()-> {
            Optional<PathPlannerPath> optionalPath = getCurrentPath();

            if (optionalPath.isEmpty()){
                for (int i = 0; i < 7; i++){ //TODO: REMOVE
                    System.out.println("none");
                }
                return none();
            }
            else {
                PathPlannerPath path = optionalPath.get();
                var pathPoints = path.getAllPathPoints();
                var translation = pathPoints.get(pathPoints.size() - 1).position;
                var rotation = path.getGoalEndState().rotation();

                pathNumber++;

                return pathfindToPose(new Pose2d(translation, rotation));
            }
        }, requirements);
    }

    // public Command pathfindToPose(Pose2d pose) {
    //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //         drivetrain.getGlobalPoseEstimate(),
    //         pose
    //     );

        
    //     // Create the path using the waypoints created above
    //     PathPlannerPath path = new PathPlannerPath(
    //         waypoints,
    //         constraints,
    //         null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
    //         new GoalEndState(0, pose.getRotation())
    //     );

    //     return AutoBuilder.followPath(path);
    // }

    public Command getAuto() {
        return Optional.ofNullable(autoChooser.selectedCommand()).orElse(none());
    }

    public void log() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
}
