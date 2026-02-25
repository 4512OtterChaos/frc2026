package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
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
    private OCXboxController driver;
    private Superstructure superstructure;

    
    private boolean autosSetup = false;

    public AutoOptions(OCDrivetrain drivetrain, Intake intake, Hood hood, Flywheel flywheel, Spindexer spindexer, FourBar fourBar, Feeder feeder, Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.hood = hood;
        this.flywheel = flywheel;
        this.spindexer = spindexer;
        this.fourBar = fourBar;
        this.feeder = feeder;
        this.superstructure = superstructure;

        SmartDashboard.putData(autoOptions);

        // AutoBuilder.configureHolonomic(
        //     ()->drive.getPose(),
        //     (resetPose)->drive.resetOdometry(resetPose),
        //     ()->drive.getChassisSpeeds(),
        //     (targetChassisSpeeds)->drive.setChassisSpeeds(targetChassisSpeeds, false, true),
        //     AutoConstants.kPathConfig,
        //     ()->drive.getIsDrivingMirrored(),
        //     drive
        // );

    }

    private void addAutoMethods(){
        NamedCommands.registerCommand("Intake", intake.setVoltageInC());
        NamedCommands.registerCommand("Shoot", parallel(superstructure.shootShotMapC(()-> Shotmap.distanceToHub(drivetrain.getGlobalPoseEstimate(), FieldUtil.kHubTrl)), drivetrain.driveFacingHub(driver)));
    }
}
