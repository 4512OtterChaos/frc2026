package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class Shotmap {
    private static InterpolatingTreeMap<Double, State> map = 
        new InterpolatingTreeMap<Double, State>(InverseInterpolator.forDouble(), (State startValue, State endValue, double t)-> startValue.interpolate(endValue, t));

    static {
        addState(Inches.of(100),Degrees.of(5), RPM.of(2800),Seconds.of(1.5));// TODO: use real values
        addState(Inches.of(200),Degrees.of(10), RPM.of(3500),Seconds.of(2));// TODO: use real values
        addState(Inches.of(300),Degrees.of(15), RPM.of(4200),Seconds.of(2.5));// TODO: use real values
    }

    private static void addState(Distance distance, Angle angle, AngularVelocity velocity, Time tof){
        map.put(distance.in(Meters), new State(angle, velocity, tof));
    }

    public static State getState(Distance distance){
        return map.get(distance.in(Meters));
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

        State(Angle angle, AngularVelocity velocity, Time tof){
            this.angle = angle;
            this.velocity = velocity;
            this.tof = tof;
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
                    Seconds.of(MathUtil.interpolate(this.getTof().in(Seconds), endValue.getTof().in(Seconds), t))
                );
            }
        }
    }

}
