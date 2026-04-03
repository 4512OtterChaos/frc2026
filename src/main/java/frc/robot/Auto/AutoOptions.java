package frc.robot.Auto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.RobotConstants.*;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FileVersionException;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    PathConstraints constraints = new PathConstraints(MetersPerSecond.of(3.5), MetersPerSecondPerSecond.of(4.5), DegreesPerSecond.of(540), DegreesPerSecondPerSecond.of(720), Volts.of(12)); //TODO: Use units class
    private Optional<PathPlannerPath> currentPath = Optional.empty();

    private List<PathPlannerPath> topBumpCyclesPaths;
    private List<PathPlannerPath> fasterTopBumpCyclesPaths;
    private List<PathPlannerPath> fasterFasterTopBumpCyclesPaths;
    private List<PathPlannerPath> topDoubleCyclePaths;
    private List<PathPlannerPath> middleDepotPaths;

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
        (chassisSpeeds) -> {
            Translation2d correction = getPathCorrection();
            ChassisSpeeds correctedDriving = new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond +correction.getX(),
            chassisSpeeds.vyMetersPerSecond + correction.getY(),
            chassisSpeeds.omegaRadiansPerSecond
            );
            // drivetrain.driveC(corrected);
            drivetrain.driveAutos(correctedDriving);
        },
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
        new EventTrigger("Shoot")
            .onTrue(superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).withTimeout(4));

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
        // autoChooser.addCmd("Left Bump Cycles", ()-> AutoBuilder.buildAuto("Top Bump Cycles"));
        // autoChooser.addCmd("Right Bump Cycles", ()-> new PathPlannerAuto("Top Bump Cycles", true));
        // autoChooser.addCmd("Faster Left Bump Cycles", ()-> AutoBuilder.buildAuto("Faster Top Bump Cycles"));
        // autoChooser.addCmd("Faster Right Bump Cycles", ()-> new PathPlannerAuto("Faster Top Bump Cycles", true));
        // autoChooser.addCmd("Faster Faster Left Bump Cycles", ()-> AutoBuilder.buildAuto("Faster Faster Top Bump Cycles"));
        // autoChooser.addCmd("Faster Faster Right Bump Cycles", ()-> new PathPlannerAuto("Faster Faster Top Bump Cycles", true));
        // autoChooser.addCmd("Left Double Cycle", ()-> AutoBuilder.buildAuto("Top Double Cycle"));
        // autoChooser.addCmd("Right Double Cycle", ()-> new PathPlannerAuto("Top Double Cycle", true));
        // autoChooser.addCmd("Middle Depot", ()-> AutoBuilder.buildAuto("Middle Depot"));
        // autoChooser.addCmd("TEST", ()-> Commands.none());

        
        try {
            topBumpCyclesPaths = List.of(PathPlannerPath.fromPathFile("Top Bump Cycles 1-2"), PathPlannerPath.fromPathFile("Top Bump Cycles 2-2"));
            fasterTopBumpCyclesPaths = List.of(PathPlannerPath.fromPathFile("Faster Top Bump Cycles 1-2"), PathPlannerPath.fromPathFile("Faster Top Bump Cycles 2-2"));
            fasterFasterTopBumpCyclesPaths = List.of(PathPlannerPath.fromPathFile("Faster Faster Top Bump Cycles 1-2"), PathPlannerPath.fromPathFile("Faster Faster Top Bump Cycles 2-2"));
            topDoubleCyclePaths = List.of(PathPlannerPath.fromPathFile("Top Double Cycle 1-2"), PathPlannerPath.fromPathFile("Top Double Cycle 2-2"));
            middleDepotPaths = List.of(PathPlannerPath.fromPathFile("Middle Depot 1-2"), PathPlannerPath.fromPathFile("Middle Depot 2-2"));
        } catch (IOException | org.json.simple.parser.ParseException | FileVersionException e) {
            topBumpCyclesPaths = List.of();
            fasterTopBumpCyclesPaths = List.of();
            fasterFasterTopBumpCyclesPaths = List.of();
            topDoubleCyclePaths = List.of();
            middleDepotPaths = List.of();
        }

        autoChooser.addCmd("Left Bump Cycles",  ()-> correctedAutoCommand("0101", topBumpCyclesPaths));
        autoChooser.addCmd("Right Bump Cycles", ()-> new PathPlannerAuto("Top Bump Cycles", true));
        autoChooser.addCmd("Faster Left Bump Cycles", ()-> AutoBuilder.buildAuto("Faster Top Bump Cycles"));
        autoChooser.addCmd("Faster Right Bump Cycles", ()-> new PathPlannerAuto("Faster Top Bump Cycles", true));
        autoChooser.addCmd("Faster Faster Left Bump Cycles", ()-> AutoBuilder.buildAuto("Faster Faster Top Bump Cycles"));
        autoChooser.addCmd("Faster Faster Right Bump Cycles", ()-> new PathPlannerAuto("Faster Faster Top Bump Cycles", true));
        autoChooser.addCmd("Left Double Cycle", ()-> AutoBuilder.buildAuto("Top Double Cycle"));
        autoChooser.addCmd("Right Double Cycle", ()-> new PathPlannerAuto("Top Double Cycle", true));
        autoChooser.addCmd("Middle Depot", ()-> AutoBuilder.buildAuto("Middle Depot"));
    }

    public Command pathfindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, constraints);
    }

    public Command pathfindToPathEnd(PathPlannerPath path) {
        Supplier<PathPoint> test = ()->{
            var pathPoints = path.getAllPathPoints();
            var lastPoint = pathPoints.get(0);
            return lastPoint;
        };
        return pathfindToPose(new Pose2d(test.get().position, test.get().rotationTarget.rotation()));
    }

    public Command followPathTracked(Optional<PathPlannerPath> path) {
        return Commands.sequence(
            Commands.runOnce(() -> currentPath = path),
            AutoBuilder.followPath(path.get())
        );
    }
    
    // public Command pathErrorCorrection() {
    //     return Commands.defer(() -> {
    //         double tolerance = 0.5;

    //         if (currentPath.isEmpty()) {return Commands.none();}

    //         Pose2d robotPose = drivetrain.getGlobalPoseEstimate();
    //         List<PathPoint> pathPoints = currentPath.get().getAllPathPoints();

    //         if (pathPoints.isEmpty()) {
    //             return Commands.none();
    //         }

    //         PathPoint closestPoint = pathPoints.stream().min((a, b) -> Double.compare(
    //                 robotPose.getTranslation().getDistance(a.position),
    //                 robotPose.getTranslation().getDistance(b.position)
    //             )).orElse(pathPoints.get(0));

    //         double error = robotPose.getTranslation().getDistance(closestPoint.position);

    //         Rotation2d targetRotation = closestPoint.rotationTarget != null
    //             ? closestPoint.rotationTarget.rotation()
    //             : robotPose.getRotation();

    //         if (error > tolerance) {
    //             System.out.println("BAD BAD BAD BADDDDD error: " +error);

    //             Pose2d targetPose = new Pose2d(
    //                 closestPoint.position,
    //                 targetRotation
    //             );
    //             return AutoBuilder.pathfindToPose(targetPose, constraints);
    //         }

    //         return Commands.none();

    //     }, Set.of(drivetrain));
    // }

    public Translation2d getPathCorrection() {
        if (currentPath.isEmpty()) return new Translation2d();
        Pose2d robotPose = drivetrain.getGlobalPoseEstimate();
        List<PathPoint> points = currentPath.get().getAllPathPoints();

        if (points.isEmpty()) {
            return new Translation2d();
        }

        PathPoint closestPoint = points.stream()
            .min((a, b) -> Double.compare(
                robotPose.getTranslation().getDistance(a.position), robotPose.getTranslation().getDistance(b.position)
            ))
            .orElse(points.get(0));

        Translation2d errorDifference =
            closestPoint.position.minus(robotPose.getTranslation());

        double tuning = 1.5; // TODO: tune and make it a tuneable number maybe

        return errorDifference.times(tuning);
    }

    /**
     * @param order 0 for path, 1 for shoot command, 2 for infinite shoot yea its dumb system
     * @param shoot for shoot command
     * @param paths paths in order
     * @return Command
     */
    public Command correctedAutoCommand(String order, List<PathPlannerPath> paths) {
        List<Command> sequence = new ArrayList<>();

        var orderPointer = 0;
        // var commandPointer = 0;
        
        for (char c : order.toCharArray()) {
            switch (c) {
                case '0' -> {
                    if (orderPointer >= paths.size()) continue;
                    PathPlannerPath path = paths.get(orderPointer);
                    orderPointer++;
                    sequence.add(followPathTracked(Optional.of(path)));
                }
                case '1' -> // if (commandPointer >= shoot.size()) continue;
                    sequence.add(superstructure.otterShootStationaryC(()-> new ChassisSpeeds()).withTimeout(4));
                case '2' -> // if (commandPointer >= shoot.size()) continue;
                    sequence.add(superstructure.otterShootStationaryC(()-> new ChassisSpeeds()));
                default -> {}
            }
        }

        Command everythingSequence = Commands.sequence(sequence.toArray(new Command[0]));


        // Command correctingBadPath = Commands.sequence(
        //     waitSeconds(0.5), // TODO: maybe tune wait time
        //     Commands.repeatingSequence(pathErrorCorrection())
        // );

        // return Commands.sequence(everythingSequence).raceWith(
        //     Commands.sequence(
        //         waitSeconds(0.5),
        //         Commands.repeatingSequence(
        //             Commands.defer(
        //                 ()-> Commands.either(
        //                     pathErrorCorrection(),
        //                     Commands.none(),
        //                     () -> shouldCorrect()
        //                 ),
        //                 Set.of(drivetrain)
        //             )
        //         )
        //     )
        // );

        return Commands.sequence(everythingSequence);
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
