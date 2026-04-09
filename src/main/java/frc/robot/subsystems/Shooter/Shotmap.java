package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.kRobotToFuelExitTrf2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.FieldUtil;

public class Shotmap {
    private static InterpolatingTreeMap<Double, Shooter.State> map = 
        new InterpolatingTreeMap<Double, Shooter.State>(InverseInterpolator.forDouble(), (Shooter.State startValue, Shooter.State endValue, double t)-> startValue.interpolate(endValue, t));

    // private static InterpolatingTreeMap<Double, Double> map2 = 
    //     new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    private static Shooter.State presetState;

    public static Distance backCorner = Meters.of(6.523654706);
    public static Distance trench = Meters.of(5.24860998);
    public static Distance nextToTower = Meters.of(4.716783828);
    public static Distance frontOfTower = Meters.of(3.55593061);
    public static Distance nearHub = Meters.of(2.342913695);
    public static Shooter.State idleState = new Shooter.State(ShooterConstants.kHoodMinAngle, ShooterConstants.kFlywheelIdleVelocity, null);

    private static Time minTof = null;
    private static Time maxTof = null;

    static {
        // 4.86 
        addState(Meters.of(2.26), Degrees.of(21), RPM.of(2550), Seconds.of(1.02));
        addState(Meters.of(3.5), Degrees.of(24), RPM.of(2950), Seconds.of(1.22));
        addState(Meters.of(5.57), Degrees.of(34), RPM.of(3425), Seconds.of(1.31));
        addState(Meters.of(14.00), Degrees.of(45), RPM.of(4500), Seconds.of(2));//TODO: not real tof
    }

    private static void addState(Distance distance, Angle angle, AngularVelocity velocity, Time tof){
        map.put(distance.in(Meters), new Shooter.State(angle, velocity, tof));
        // map2.put(distance.in(Meters)/tof.in(Seconds), distance.in(Meters));
        if (minTof == null || tof.in(Seconds) < minTof.in(Seconds)){
            minTof = tof;
        }
        if (maxTof == null || tof.in(Seconds) > maxTof.in(Seconds)){
            maxTof = tof;
        }
    }

    // public static double horizontalVelocityToEffectiveDistance(double velocity){
    //     return map2.get(velocity);
    // }

    /** Get the shooter-relative fuel velocities (x and z) for the state at given distance */
    public static Translation3d getRelativeFuelVels(Angle hoodAngle, AngularVelocity flywheelVel) {
        var refDist = Meters.of(3.5);
        var refState = getState(refDist);
        double refHoodAngleRads = refState.getAngle().in(Radians);
        double refHorizVelMeters = refDist.div(refState.getTof()).in(MetersPerSecond);
        double refLinearVelMeters = refHorizVelMeters / Math.sin(refHoodAngleRads);

        double linearVelMeters = flywheelVel.div(refState.getVelocity()).magnitude() * refLinearVelMeters;
        double hoodAngleRads = hoodAngle.in(Radians);
        return new Translation3d(
            linearVelMeters * Math.sin(hoodAngleRads),
            0,
            linearVelMeters * Math.cos(hoodAngleRads));
    }

    public static Shooter.State getState(Distance distance){
        return map.get(distance.in(Meters));
    }

    public static Distance distanceToHub(Pose2d robotPose) {
        return distanceToTarget(robotPose, FieldUtil.kHubTrl);
    }  

    public static Distance distanceToTarget(Pose2d robotPose, Translation2d targetTranslation) {
        double meters = robotPose.getTranslation().getDistance(targetTranslation);
        return Meters.of(meters);
    }   

    public static Angle getAngle(Distance distance){
        return getState(distance).getAngle();
    }

    public static AngularVelocity getVelocity(Distance distance){
        return getState(distance).getVelocity();
    }

    public static Time tof(Distance distance){
        return getState(distance).getTof();
    }
    
    public static Time getMinTof() {
        return minTof;
    }

    public static Time getMaxTof() {
        return maxTof;
    }
    
    public static void setPresetState(Distance dist) {
        presetState = getState(dist);
    }
    
    public static Shooter.State getPresetState() {
        return presetState;
    } 

    /**
     * Find the field-relative angle of the robot for the shooter to align with the target.
     * @param robotPose
     * @param target
     * @return
     */
    public static Rotation2d getFieldRelTargetFacingAngle(Pose2d robotPose, Translation2d target) { // TODO: check
        // find the angle needed to point the chassis at the target
        Rotation2d robotFaceTarget = target.minus(robotPose.getTranslation()).getAngle();

        // using that angle, adjust the target by the offset from the robot to the shooter exit
        Translation2d adjTarget = target.plus(
            kRobotToFuelExitTrf2d.getTranslation().rotateBy(robotFaceTarget)
        );

        // the adjusted target is the point the robot needs to face to point the shooter at the target
        // (plus a 180 deg rotation)
        return adjTarget.minus(robotPose.getTranslation()).getAngle().plus(Rotation2d.k180deg);
    }

    /**
     * Get the angular velocity needed to keep the shooter facing the target, given the robot's current linear velocity.
     * @param robotPose
     * @param target
     * @param fieldSpeeds Field-relative chassis speeds of the robot
     * @return
     */
    public static Rotation2d targetFacingOmega(Pose2d robotPose, Translation2d target, ChassisSpeeds fieldSpeeds) {
        Translation2d shooterTrl = robotPose.plus(kRobotToFuelExitTrf2d).getTranslation();
        Translation2d relTarget = target.minus(shooterTrl);
        // mathematically, the target-facing angle is arctan(relTargetY / relTargetX)
        // we want to find its derivative with respect to time, which is the target-facing omega
        // using the quotient rule, we get:
        // omega = (relTargetX * relTargetY' - relTargetY * relTargetX') / (relTargetX^2 + relTargetY^2)
        
        double x = relTarget.getX();
        double y = relTarget.getY();

        double distSq = x*x + y*y;
        if (distSq < 1e-6) {
            return Rotation2d.kZero;
        }

        double vX = fieldSpeeds.vxMetersPerSecond;
        double vY = fieldSpeeds.vyMetersPerSecond;
        return Rotation2d.fromRadians((x * vY - y * vX) / distSq);
    }

}
