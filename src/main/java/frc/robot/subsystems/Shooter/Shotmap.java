package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.FieldUtil;

public class Shotmap {

    public static Distance distanceToHub(Pose2d robotPose , Translation2d kHubPos) {
        double meters = robotPose.getTranslation().getDistance(kHubPos);
        return Meters.of(meters);
    }   

    record DistanceHeightKey(double distance, double height) {}

    private static InterpolatingTreeMap<DistanceHeightKey, State> map = 
        new InterpolatingTreeMap<DistanceHeightKey, State>(InverseInterpolator.forDouble(), (State startValue, State endValue, double t)-> startValue.interpolate(endValue, t));

        
    static {
        addState(Inches.of(-100),Degrees.of(5), RPM.of(2800),Seconds.of(1.5), FieldUtil.kHubHeight);// TODO: use real values
        addState(Inches.of(-200),Degrees.of(10), RPM.of(3500),Seconds.of(2), FieldUtil.kHubHeight);// TODO: use real values
        addState(Inches.of(-300),Degrees.of(15), RPM.of(4200),Seconds.of(2.5), FieldUtil.kHubHeight);// TODO: use real values
        addState(Inches.of(100),Degrees.of(5), RPM.of(2800),Seconds.of(1.5), FieldUtil.kHubHeight);// TODO: use real values
        addState(Inches.of(200),Degrees.of(10), RPM.of(3500),Seconds.of(2), FieldUtil.kHubHeight);// TODO: use real values
        addState(Inches.of(300),Degrees.of(15), RPM.of(4200),Seconds.of(2.5), FieldUtil.kHubHeight);// TODO: use real values
    }

    private static Double minKeyMeters = null;
    private static Double maxKeyMeters = null;

    private static void addState(Distance distance, Angle angle, AngularVelocity velocity, Time tof, Distance height){
        double key = distance.in(Meters);
        map.put(key, new State(angle, velocity, tof, height));

        if (minKeyMeters == null || key < minKeyMeters) minKeyMeters = key;
        if (maxKeyMeters == null || key > maxKeyMeters) maxKeyMeters = key;
    }

    public static State getState(Distance distance){
        double key = distance.in(Meters);

        // Clamp into map domain so interpolation always has bounds
        if (minKeyMeters != null && maxKeyMeters != null) {
            key = MathUtil.clamp(key, minKeyMeters, maxKeyMeters);
        }

        return map.get(key);
    }

    public static Distance distanceToHub3D(Pose2d robotPose, double robotHeight, Translation2d hubPos, double hubHeight) {
        double dx = hubPos.getX() - robotPose.getX();
        double dy = hubPos.getY() - robotPose.getY();
        double dz = hubHeight - robotHeight; // vertical difference
        double meters = Math.sqrt(dx*dx + dy*dy + dz*dz);
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

    public static class State implements Interpolatable<State>{
        private Angle angle; 
        private AngularVelocity velocity; 
        private Time tof;
        private Distance height;

        State(Angle angle, AngularVelocity velocity, Time tof, Distance height){
            this.angle = angle;
            this.velocity = velocity;
            this.tof = tof;
            this.height = height;
        }

        public Angle getAngle(){
            return angle; 
        }

        public AngularVelocity getVelocity(){
            return velocity;
        }

        public Time getTof() {
            return tof;
        }

        public Distance getHeight() {
            return height;
        }

        @Override
        public State interpolate(State endValue, double t) {
            if (t <= 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                return new State(
                    Degrees.of(MathUtil.interpolate(this.getAngle().in(Degrees), endValue.getAngle().in(Degrees), t)),                
                    RPM.of(MathUtil.interpolate(this.getVelocity().in(RPM), endValue.getVelocity().in(RPM), t)),
                    Seconds.of(MathUtil.interpolate(this.getTof().in(Seconds), endValue.getTof().in(Seconds), t)),
                    Meters.of(MathUtil.interpolate(this.getHeight().in(Meters), endValue.getHeight().in(Meters), t))
                );
            }
        }
    }
}
