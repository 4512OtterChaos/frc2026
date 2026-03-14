package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldUtil {
    public static final Distance kFieldWidth = Inches.of(317.7);
    public static final Distance kFieldLength = Inches.of(651.2);

    public static final Translation2d kAllianceZoneMax = new Translation2d(
        Inches.of(182.1).in(Meters),
        kFieldWidth.in(Meters)
    );

    public static final Translation2d kBottomTrenchZoneMin = new Translation2d(
        Inches.of(182.1).minus(Inches.of(50)).in(Meters),
        0
    );
    public static final Translation2d kBottomTrenchZoneMax = new Translation2d(
        Inches.of(182.1).plus(Inches.of(50)).in(Meters),
        Inches.of(47).in(Meters)
    );

    public static final Translation2d kTopTrenchZoneMin = new Translation2d(
        Inches.of(182.1).minus(Inches.of(50)).in(Meters),
        kFieldWidth.minus(Inches.of(47)).in(Meters)
    );
    public static final Translation2d kTopTrenchZoneMax = new Translation2d(
        Inches.of(182.1).plus(Inches.of(50)).in(Meters),
        kFieldWidth.in(Meters)
    );

    public static final Translation2d kNeutralZoneMin = new Translation2d(
        Inches.of(158.6).in(Meters),
        0
    );
    public static final Translation2d kNeutralZoneMax = new Translation2d(
        kFieldLength.minus(Inches.of(158.6)).in(Meters),
        kFieldWidth.in(Meters)
    );
 
    public static final Translation2d kHubTrl = new Translation2d(
        Inches.of(182.1).in(Meters),
        kFieldWidth.div(2).in(Meters)
    );

    public static final Translation2d kLeftNeutralSetpoint = new Translation2d( // TODO:tune netral zone shooting setpoints
        kFieldLength.div(5).in(Meters),
        kFieldWidth.div(5).in(Meters)
    );

    public static final Translation2d kRightNeutralSetpoint = new Translation2d( // TODO:tune netral zone shooting setpoints
        kFieldLength.div(5).in(Meters),
        kFieldWidth.minus(kFieldWidth.div(5)).in(Meters)
    );

    public static final ArrayList<Pose2d> kSetpoints;
    static{
        kSetpoints = new ArrayList<Pose2d>();
        kSetpoints.add(new Pose2d(kLeftNeutralSetpoint, Rotation2d.kZero));
        kSetpoints.add( new Pose2d(kRightNeutralSetpoint, Rotation2d.kZero));
    }

    public static final Distance kHubHeight = Inches.of(72);
    public static final Distance kHubWidth = Inches.of(47);
}
