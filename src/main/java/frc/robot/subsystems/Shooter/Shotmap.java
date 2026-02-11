package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.gravity;
import static frc.robot.subsystems.Shooter.ShooterConstants.kWheelRadius;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.FieldUtil;

public class Shotmap {

    public static Distance distanceToHub(Pose2d robotPose , Translation2d kHubPos) {
        double meters = robotPose.getTranslation().getDistance(kHubPos);
        return Meters.of(meters);
    }   
    
    private static InterpolatingTreeMap<Double, State> map = 
        new InterpolatingTreeMap<Double, State>(InverseInterpolator.forDouble(), (State startValue, State endValue, double t)-> startValue.interpolate(endValue, t));

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
    
    public static State SOTFLogic(Pose2d robotPose, Translation2d hubPos, Translation2d robotFieldVelocity) {
            // 1. Distance
            Distance dist = distanceToHub(robotPose, hubPos);
            State base = getState(dist);

            // 2. Direction to goal
            Translation2d toHub = hubPos.minus(robotPose.getTranslation()).div(dist.in(Meters));

            // 3. Base horizontal velocity
            double vHorizIdeal = getHorizontalVelocity(dist).in(MetersPerSecond);
            Translation2d vTarget = toHub.times(vHorizIdeal);

            // 4. Subtract robot velocity
            Translation2d vShot = vTarget.minus(robotFieldVelocity);
            double newHorizSpeed = vShot.getNorm();

            // 5. Recompute pitch using vertical component
            double vTotal = rpmToLinear(base.getVelocity());
            double vVert = vTotal * Math.sin(base.getAngle().in(Radians));
            double newPitch = Math.atan2(vVert, newHorizSpeed);

            // 6. Build new state
            return new State(
                Radians.of(newPitch),
                linearToRPM(Math.hypot(newHorizSpeed, vVert)),
                base.getTof(),
                base.getHeight()
            );
        }

    public static Angle getAngle(Distance distance){
        return getState(distance).getAngle();
    }

    // public static Angle setAngle(LinearVelocity velocity, Distance distance, Distance height) {
    //     double value = (velocity.in(MetersPerSecond) * velocity.in(MetersPerSecond)
    //                 + Math.sqrt(Math.pow(velocity.in(MetersPerSecond), 4) - gravity * (gravity * distance.in(Meters) * distance.in(Meters) + 2 * height.in(Meters) * velocity.in(MetersPerSecond) * velocity.in(MetersPerSecond)))
    //                 / (gravity * distance.in(Meters))
    //                 );
    //         double angle = Math.atan(value);
    //         return Degrees.of(angle);
    // }

    public static double rpmToLinear(AngularVelocity rpm) {
        double wheelRadius = kWheelRadius.in(Meters);
        return rpm.in(RPM) * 2.0 * Math.PI * wheelRadius / 60.0;
    }

    public static AngularVelocity linearToRPM(double metersPerSecond) {
        double wheelRadius = kWheelRadius.in(Meters);
        double rpm = metersPerSecond * 60.0 / (2.0 * Math.PI * wheelRadius);
        return RPM.of(rpm);
    }

    public static LinearVelocity getHorizontalVelocity(Distance distance) {
        State s = getState(distance);
        

        // Convert RPM -> linear exit speed (you probably already have this somewhere)
        double vTotal = rpmToLinear(s.getVelocity()); // m/s
        double theta = s.getAngle().in(Radians);

        double vHoriz = vTotal * Math.cos(theta);
        return MetersPerSecond.of(vHoriz);
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

        public State(Angle angle, AngularVelocity velocity, Time tof, Distance height){
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
