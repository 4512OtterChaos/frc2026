package frc.robot.Auto;

import static frc.robot.util.RobotConstants.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.util.Units;

public class AutoConstants {
    // our maximum speeds/accelerations during auto -- NOTE: actually only defined in PathPlanner now
    public static final double kMaxLinearSpeed = Units.feetToMeters(15);
    // public static final double kMaxLinearAcceleration = Units.feetToMeters(18);
    // public static final double kMaxAngularSpeed = Units.rotationsToRadians(1.75);
    // public static final double kMaxAngularAcceleration = Units.rotationsToRadians(3);

    public static final double kThetaPositionTolerance = Units.degreesToRadians(3.5);
    public static final double kThetaVelocityTolerance = Units.degreesToRadians(10);

    // The max speed used here is for ensuring rotating while translating doesnt command more speed than is possible
    public static final PPHolonomicDriveController kPathConfig = new PPHolonomicDriveController( //TODO: tune later >:(
        new PIDConstants(3, 0, 0),
        new PIDConstants(3, 0, 0)
    );

    RobotConfig robotConfig = new RobotConfig(kRobotWeight, kMOI, kModuleConfig, FL, FR, BL, BR);
}
