package frc.robot.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.util.RobotConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;

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

    public AutoOptions(OCDrivetrain drivetrain, Intake intake, Hood hood, Flywheel flywheel, Spindexer spindexer, FourBar fourBar, Climber climber, Feeder feeder, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.hood = hood;
        this.flywheel = flywheel;
        this.spindexer = spindexer;
        this.fourBar = fourBar;
        this.feeder = feeder;
        this.climber = climber;
        this.superstructure = superstructure;

        SmartDashboard.putData(autoOptions);

        AutoBuilder.configure(
            ()-> drivetrain.getGlobalPoseEstimate(), 
            (pose)-> drivetrain.resetPose(pose), 
            ()-> drivetrain.getState().Speeds, 
            (chassisSpeeds)-> drivetrain.drive(chassisSpeeds), 
            AutoConstants.kPathConfig, 
            robotConfig,
            ()-> drivetrain.driveMirror(),
            drivetrain, intake, hood, flywheel, spindexer, fourBar, feeder, climber
        );

        addAutoMethods();
    }   

    public Command shootC() {
        return Commands.parallel(superstructure.shootShotMapC(()-> Shotmap.distanceToHub(drivetrain.getGlobalPoseEstimate(), FieldUtil.kHubTrl)), drivetrain.driveFacingHub(driver));
    }

    private void addAutoMethods(){
        NamedCommands.registerCommand("Intake", intake.setVoltageInC());
        NamedCommands.registerCommand("Shoot", shootC());
    }

    public void periodic() {
        if (!autosSetup && !DriverStation.getAlliance().isEmpty()){
            autoOptions.setDefaultOption("none", resetInitialOdomC());
            addTopShootClimbOption();
            addBottomShootClimbOption();
            addTopDepoClimbOption();
            autosSetup = true;
        }
    }

    public void addTopShootClimbOption(){
        autoOptions.addOption("Shoot",
            AutoBuilder.buildAuto("Shoot")
        );
        autoOptions.addOption("ClimberUp",
            AutoBuilder.buildAuto("ClimberUp")
        );
        autoOptions.addOption("ClimberDown",
            AutoBuilder.buildAuto("ClimberDown")
        );
    }
    
    public void addBottomShootClimbOption(){
        autoOptions.addOption("ClimberUp",
            AutoBuilder.buildAuto("ClimberUp")
        );
        autoOptions.addOption("ClimberDown",
            AutoBuilder.buildAuto("ClimberDown")
        );
        autoOptions.addOption("Shoot",
            AutoBuilder.buildAuto("Shoot")
        );
    }
    
    public void addTopDepotClimbOption(){
        autoOptions.addOption("Intake",
            AutoBuilder.buildAuto("Intake")
        );
        autoOptions.addOption("Shoot",
            AutoBuilder.buildAuto("Shoot")
        );
        autoOptions.addOption("ClimberUp",
            AutoBuilder.buildAuto("ClimberUp")
        );
        autoOptions.addOption("ClimberDown",
            AutoBuilder.buildAuto("ClimberDown")
        );
    }
}
