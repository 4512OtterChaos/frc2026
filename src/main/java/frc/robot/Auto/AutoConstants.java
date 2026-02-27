package frc.robot.Auto;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.RobotConstants.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Drivetrain.TunerConstants;

public class AutoConstants {

    private OCDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // our maximum speeds/accelerations during auto -- NOTE: actually only defined in PathPlanner now
    public static final double kMaxLinearSpeed = Units.feetToMeters(15);
    // public static final double kMaxLinearAcceleration = Units.feetToMeters(18);
    // public static final double kMaxAngularSpeed = Units.rotationsToRadians(1.75);
    // public static final double kMaxAngularAcceleration = Units.rotationsToRadians(3);

    // pose PID control. 1 meter error in x = kP meters per second in target x velocity 
    public static final double kPXController = 3;
    public static final double kPYController = 3;
    public static final double kPThetaController = 5;
    public static final double kDThetaController = 0.1;

    // constraints for the theta controller on velocity (omega) and acceleration (alpha)
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        Units.rotationsToRadians(1.5), //   TODO: Tune
        Units.rotationsToRadians(3) // TODO: Tune
    );

    public static final double kThetaPositionTolerance = Units.degreesToRadians(3.5);
    public static final double kThetaVelocityTolerance = Units.degreesToRadians(10);

    // The max speed used here is for ensuring rotating while translating doesnt command more speed than is possible
    public static final PPHolonomicDriveController kPathConfig = new PPHolonomicDriveController( //TODO: tune later >:(
        new PIDConstants(7, 0, 0),
        new PIDConstants(9, 0, 0)
    );

    RobotConfig robotConfig = new RobotConfig(kRobotWeight, kMOI, kModuleConfig, FL, FR, BL, BR);
}
