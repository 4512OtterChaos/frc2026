package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
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

    private static InterpolatingTreeMap<Double, Double> map2 = 
        new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    private static Shooter.State presetState;

    public static Distance backCorner = Meters.of(6.523654706);
    public static Distance trench = Meters.of(5.24860998);
    public static Distance nextToTower = Meters.of(4.716783828);
    public static Distance frontOfTower = Meters.of(3.55593061);
    public static Distance nearHub = Meters.of(2.342913695);
    public static Shooter.State idleState = new Shooter.State(Degrees.of(ShooterConstants.hoodMinAngle.get()), RPM.of(ShooterConstants.flywheelIdleRPM.get()), null);

    private static Time minTof = null;
    private static Time maxTof = null;

    static {
        // 4.86 
        addState(Meters.of(2.26), Degrees.of(0), RPM.of(2800), Seconds.of(1.02));
        addState(Meters.of(3.5), Degrees.of(3), RPM.of(3200), Seconds.of(1.22));
        addState(Meters.of(5.57), Degrees.of(13), RPM.of(3600), Seconds.of(1.31));//TODO: not measured for the correct pose, instead 4.86 meters
    }

    public static void periodic() {
        changeTunable();
    }

    private static void addState(Distance distance, Angle angle, AngularVelocity velocity, Time tof){
        map.put(distance.in(Meters), new Shooter.State(angle, velocity, tof));
        map2.put(distance.in(Meters)/tof.in(Seconds), distance.in(Meters));
        if (minTof == null || tof.in(Seconds) < minTof.in(Seconds)){
            minTof = tof;
        }
        if (maxTof == null || tof.in(Seconds) > maxTof.in(Seconds)){
            maxTof = tof;
        }
    }

    public static double horizontalVelocityToEffectiveDistance(double velocity){
        return map2.get(velocity);
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

        //shoot on da fly 
    public static Rotation2d newTargetAngle(Pose2d robotPose, ChassisSpeeds speed, Translation2d target) { // TODO: check
        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d trlToTarget = target.minus(robotPosition);

        return trlToTarget.getAngle();
    }



    public static void changeTunable() {
        targetMultiplier.poll();
    }

}
