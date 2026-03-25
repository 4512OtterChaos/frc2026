package frc.robot.util;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class TunableUnits {
    public static class TunableDistance extends TunableUnitBase<Distance, DistanceUnit>{
        public TunableDistance(String key, Distance defaultValue, DistanceUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableDistance(String key, Distance defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableTime extends TunableUnitBase<Time, TimeUnit>{
        public TunableTime(String key, Time defaultValue, TimeUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableTime(String key, Time defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableAngle extends TunableUnitBase<Angle, AngleUnit>{
        public TunableAngle(String key, Angle defaultValue, AngleUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableAngle(String key, Angle defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableLinearVelocity extends TunableUnitBase<LinearVelocity, LinearVelocityUnit>{
        public TunableLinearVelocity(String key, LinearVelocity defaultValue, LinearVelocityUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableLinearVelocity(String key, LinearVelocity defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableAngularVelocity extends TunableUnitBase<AngularVelocity, AngularVelocityUnit>{
        public TunableAngularVelocity(String key, AngularVelocity defaultValue, AngularVelocityUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableAngularVelocity(String key, AngularVelocity defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableLinearAcceleration extends TunableUnitBase<LinearAcceleration, LinearAccelerationUnit>{
        public TunableLinearAcceleration(String key, LinearAcceleration defaultValue, LinearAccelerationUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableLinearAcceleration(String key, LinearAcceleration defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableAngularAcceleration extends TunableUnitBase<AngularAcceleration, AngularAccelerationUnit>{
        public TunableAngularAcceleration(String key, AngularAcceleration defaultValue, AngularAccelerationUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableAngularAcceleration(String key, AngularAcceleration defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableMass extends TunableUnitBase<Mass, MassUnit>{
        public TunableMass(String key, Mass defaultValue, MassUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableMass(String key, Mass defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableVoltage extends TunableUnitBase<Voltage, VoltageUnit>{
        public TunableVoltage(String key, Voltage defaultValue, VoltageUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableVoltage(String key, Voltage defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableCurrent extends TunableUnitBase<Current, CurrentUnit>{
        public TunableCurrent(String key, Current defaultValue, CurrentUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableCurrent(String key, Current defaultValue){
            super(key, defaultValue);
        }
    }
    public static class TunableMomentOfInertia extends TunableUnitBase<MomentOfInertia, MomentOfInertiaUnit>{
        public TunableMomentOfInertia(String key, MomentOfInertia defaultValue, MomentOfInertiaUnit preferedUnit){
            super(key, defaultValue, preferedUnit);
        }
        public TunableMomentOfInertia(String key, MomentOfInertia defaultValue){
            super(key, defaultValue);
        }
    }
}
