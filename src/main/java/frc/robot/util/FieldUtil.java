package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

public class FieldUtil {
    public static final Distance kFieldWidth = Inches.of(317.7);
    public static final Distance kFieldLength = Inches.of(651.2);
    public static final Distance kHubBarrier = Inches.of(40);
 
    public static final Translation2d kHubTrl = new Translation2d(
        Inches.of(182.1).in(Meters),
        kFieldWidth.div(2).in(Meters)
    );

    public static final Distance kHubHeight = Inches.of(72);
    public static final Distance kHubWidth = Inches.of(47);

    public static final double kShooterHeight = 22; 
}
