package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    static {
        addState(Inches.of(100),Degrees.of(5), RPM.of(2800),Seconds.of(1.5));// TODO: use real values
        addState(Inches.of(200),Degrees.of(10), RPM.of(3500),Seconds.of(2));// TODO: use real values
        addState(Inches.of(300),Degrees.of(15), RPM.of(4200),Seconds.of(2.5));// TODO: use real values
    }

    public static void periodic() {
        changeTunable();
    }

    private static void addState(Distance distance, Angle angle, AngularVelocity velocity, Time tof){
        map.put(distance.in(Meters), new Shooter.State(angle, velocity, tof));
    }

    public static Shooter.State getState(Distance distance){
        return map.get(distance.in(Meters));
    }

    public static Distance distanceToHub(Pose2d robotPose) {
        double meters = robotPose.getTranslation().getDistance(FieldUtil.kHubTrl);
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

        //shoot on da fly 
    public static Rotation2d newTargetAngle(Pose2d robotPose, ChassisSpeeds speed) { // TODO: check
        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d fakeTarget = FieldUtil.kHubTrl.minus(robotPosition);

        double distanceMeters = fakeTarget.getNorm();

        Shooter.State state = Shotmap.getState(Meters.of(distanceMeters));
        
        double tof = state.getTof().in(Seconds);
        Translation2d velocityOffset = new Translation2d(speed.vxMetersPerSecond * tof * targetMultiplier.get(), speed.vyMetersPerSecond * targetMultiplier.get());
        Translation2d newTargetAngle = fakeTarget.minus(velocityOffset);

        return newTargetAngle.getAngle();
    }

    public static void changeTunable() {
        targetMultiplier.poll();
    }

}
