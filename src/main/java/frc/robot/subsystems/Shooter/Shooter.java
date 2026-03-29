package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.OCTrigger;

public class Shooter extends SubsystemBase {
    private TalonFX fwLeftMotor = new TalonFX(kLeftMotorID);
    private TalonFX fwRightMotor = new TalonFX(kRightMotorID);

    private TalonFX hMotor = new TalonFX(kHoodMotorID);

    private AngularVelocity targetVelocity = RPM.of(0); // flywheel
    private Angle targetAngle = kHoodMinAngle; //hood

    public final Trigger isIdle = new Trigger(() -> targetVelocity.isNear(kFlywheelIdleVelocity, 0.01) && targetAngle.isNear(kHoodMinAngle, 0.01));
    public final Trigger isUpToSpeed = OCTrigger.debounce(
        OCTrigger.debounce(
            new Trigger(() -> getFlywheelVelocity().isNear(targetVelocity, velocityTolerance.get())).and(isIdle.negate()),
            () -> flywheelDebounceTime.in(Seconds),
            DebounceType.kRising
        ),
        () -> flywheelDebounceTime.in(Seconds) * 2,
        DebounceType.kFalling
    );
    public final Trigger isAtAngle = OCTrigger.debounce(
        OCTrigger.debounce(
            new Trigger(() -> getHoodAngle().isNear(targetAngle, angleTolerance.get())).and(isIdle.negate()),
            () -> hoodDebounceTime.in(Seconds),
            DebounceType.kRising
        ),
        () -> hoodDebounceTime.in(Seconds) * 2,
        DebounceType.kFalling
    );

    private final StatusSignal<Angle> fwPositionStatus = fwLeftMotor.getPosition();
    private final StatusSignal<AngularVelocity> fwVelocityStatus = fwLeftMotor.getVelocity();
    private final StatusSignal<Voltage> fwVoltageStatus = fwLeftMotor.getMotorVoltage();
    private final StatusSignal<Current> fwStatorStatus = fwLeftMotor.getStatorCurrent();
    private final StatusSignal<Double> fwDutyCycleStatus = fwLeftMotor.getDutyCycle();

    private final StatusSignal<Angle> hPositionStatus = hMotor.getPosition();
    private final StatusSignal<AngularVelocity> hVelocityStatus = hMotor.getVelocity();
    private final StatusSignal<Voltage> hVoltageStatus = hMotor.getMotorVoltage();
    private final StatusSignal<Current> hStatorStatus = hMotor.getStatorCurrent();

    private final MotionMagicVoltage mmHoodRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final VelocityVoltage velocityrequest = new VelocityVoltage(0);

    public Shooter() {
        // FLYWHEEL
        fwLeftMotor.getConfigurator().apply(kFlywheelConfig);
        fwRightMotor.getConfigurator().apply(kFlywheelConfig);
        fwRightMotor.setControl(new Follower(fwLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        fwPositionStatus.setUpdateFrequency(100);
        fwDutyCycleStatus.setUpdateFrequency(100);
        fwVelocityStatus.setUpdateFrequency(100);
        fwVoltageStatus.setUpdateFrequency(100);
        fwStatorStatus.setUpdateFrequency(50);

        // HOOD
        hMotor.getConfigurator().apply(kHoodConfig);
        
        hPositionStatus.setUpdateFrequency(100);
        hVelocityStatus.setUpdateFrequency(100);
        hVoltageStatus.setUpdateFrequency(100);
        hStatorStatus.setUpdateFrequency(50);

        ParentDevice.optimizeBusUtilizationForAll(fwLeftMotor, fwRightMotor, hMotor);

        SmartDashboard.putData("4) Shooter/Subsystem", this);

        resetAngle(kHoodMinAngle);
        if (Utils.isSimulation()) {
            resetAngle(Degrees.of(0));
        }
    }

    @Override 
    public void periodic() {
        // HOOD PERIODIC
        BaseStatusSignal.refreshAll(
                hPositionStatus,
                hVelocityStatus,
                hVoltageStatus,
                hStatorStatus,

                fwPositionStatus,
                fwVelocityStatus,
                fwVoltageStatus,
                fwStatorStatus);

        hMotor.setControl(mmHoodRequest.withPosition(targetAngle));

        // FLYWHEEL PERIODIC
        fwLeftMotor.setControl(velocityrequest.withVelocity(targetVelocity));

        changeTunable();
        log();
    }

    // HOOD
    public Angle getHoodAngle() {
       return hPositionStatus.getValue();
    }

    public AngularVelocity getHoodVelocity() {
        return hVelocityStatus.getValue();
    }

    public Voltage getHoodVoltage() {
        return hVoltageStatus.getValue();
    }

    public Current getHoodCurrent() {
        return hStatorStatus.getValue();
    }

    public void resetAngle(Angle angle) {
        hMotor.setPosition(angle);
    }

    public Angle getHoodTargetAngle() {
        return targetAngle;
    }
    public void setHoodVoltage(double voltage) {
        hMotor.setVoltage(voltage);
    }

    public void setAngle(Angle angle) {
        angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), kHoodMinAngle.in(Degrees), kHoodMaxAngle.in(Degrees)));
        targetAngle = angle;
    }

    // FLYWHEEL
    public Angle getFlywheelAngle() {
       return fwPositionStatus.getValue();
    }

    public AngularVelocity getFlywheelVelocity() {
        return fwVelocityStatus.getValue();
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public Voltage getFlywheelVoltage() {
        return fwVoltageStatus.getValue();
    }

    public Current getFlywheelCurrent() {
        return fwStatorStatus.getValue();
    }

    public void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
    }
    
    //OVERALL
    public void setState(State state) {
        setState(state.getAngle(), state.getVelocity());
    }    

    public void setState(Angle angle, AngularVelocity velocity) {
        setAngle(angle);
        setVelocity(velocity);
    }
    
    public void setIdle(){
        setState(kHoodMinAngle, kFlywheelIdleVelocity);
    }
    
    public Command setIdleC(){
        return setStateC(kHoodMinAngle, kFlywheelIdleVelocity).withName("Set Idle");
    }

    public Command setStateC(State state) {
        return setStateC(state.getAngle(), state.getVelocity());
    }    
    
    public Command setStateC(Angle angle, AngularVelocity velocity) {
        return run(() -> setState(angle, velocity)).withName("Shoot: " + angle.in(Degrees) + " degrees, " + "Shoot: " + velocity.in(RPM) + " RPM");
    }  

    /** positive angular velocity = backspin */
    public Pair<LinearVelocity, AngularVelocity> getFuelExitVelSpin() {
        double flywheelRPS = getFlywheelVelocity().in(RotationsPerSecond);
        double tangentialBottom = flywheelRPS * ShooterConstants.kWheelDiameter.in(Meters) * Math.PI;
        double tangentialTop = flywheelRPS * ShooterConstants.kBackRollerRatio * ShooterConstants.kBackRollerDiameter.in(Meters) * Math.PI;
        return Pair.of(
            MetersPerSecond.of((tangentialBottom + tangentialTop) / 2),
            RadiansPerSecond.of((tangentialBottom - tangentialTop) / Centimeter.of(15).in(Meters)));
    }

    public Pose3d getFuelExitPose(Pose2d robotPose) {
        return new Pose3d(robotPose)
            .plus(kRobotToFuelExitTrf3d)
            .plus(new Transform3d(Translation3d.kZero, new Rotation3d(Degrees.of(0), Degrees.of(-90).plus(getHoodAngle()), Degrees.of(0))));
    }

    public void changeTunable() {
        // hoodMinAngle.poll();
        // hoodMaxAngle.poll();
        hoodDebounceTime.poll();
        angleTolerance.poll();
        hoodkP.poll();
        // hoodkI.poll();
        // hoodkD.poll();
        // hoodkG.poll();
        // hoodkS.poll();
        // hoodkV.poll();
        // hoodkA.poll();
        hoodCruiseVelocity.poll();
        hoodAcceleration.poll();

        // flywheelIdleVelocity.poll();
        flywheelDebounceTime.poll();
        velocityTolerance.poll();
        flywheelkP.poll();
        // flywheelkI.poll();
        // flywheelkD.poll();
        // flywheelkS.poll();
        flywheelkV.poll();
        // flywheelkA.poll();

        int hash = hashCode();

        if (hoodkP.hasChanged(hash) /*|| hoodkI.hasChanged(hash) || hoodkD.hasChanged(hash) || hoodkG.hasChanged(hash) || hoodkS.hasChanged(hash) || hoodkV.hasChanged(hash) || hoodkA.hasChanged(hash)*/ || flywheelkP.hasChanged(hash) /*|| flywheelkI.hasChanged(hash) || flywheelkD.hasChanged(hash) || flywheelkS.hasChanged(hash)*/ || flywheelkV.hasChanged(hash) /*|| flywheelkA.hasChanged(hash)*/) {
            kHoodConfig.Slot0.kP = hoodkP.get();
            // kHoodConfig.Slot0.kI = hoodkI.get();
            // kHoodConfig.Slot0.kD = hoodkD.get();
            // kHoodConfig.Slot0.kG = hoodkG.get();
            // kHoodConfig.Slot0.kS = hoodkS.get();
            // kHoodConfig.Slot0.kV = hoodkV.get();
            // kHoodConfig.Slot0.kA = hoodkA.get();
            hMotor.getConfigurator().apply(kHoodConfig.Slot0);

            kFlywheelConfig.Slot0.kP = flywheelkP.get();
            // kFlywheelConfig.Slot0.kI = flywheelkI.get();
            // kFlywheelConfig.Slot0.kD = flywheelkD.get();
            // kFlywheelConfig.Slot0.kS = flywheelkS.get();
            kFlywheelConfig.Slot0.kV = flywheelkV.get();
            // kFlywheelConfig.Slot0.kA = flywheelkA.get();
            fwLeftMotor.getConfigurator().apply(kFlywheelConfig.Slot0);
        }

        if (hoodCruiseVelocity.hasChanged(hash) || hoodAcceleration.hasChanged(hash)) {
            kHoodConfig.MotionMagic.MotionMagicCruiseVelocity = hoodCruiseVelocity.get();
            kHoodConfig.MotionMagic.MotionMagicAcceleration = hoodAcceleration.get();
            hMotor.getConfigurator().apply(kHoodConfig.MotionMagic);
        }

    }

    public void log() {        
        SmartDashboard.putNumber("4) Shooter/Hood Angle", getHoodAngle().in(Degrees));
        SmartDashboard.putNumber("4) Shooter/Target Hood Angle", targetAngle.in(Degrees));
        SmartDashboard.putNumber("4) Shooter/Hood/Voltage", getHoodVoltage().in(Volts));
        SmartDashboard.putNumber("4) Shooter/Hood/Current", getHoodCurrent().in(Amps));
        SmartDashboard.putBoolean("4) Shooter/At Angle", isAtAngle.getAsBoolean());

        SmartDashboard.putNumber("4) Shooter/Flywheel RPM", getFlywheelVelocity().in(RPM));
        SmartDashboard.putNumber("4) Shooter/Target Flywheel RPM", targetVelocity.in(RPM));
        SmartDashboard.putNumber("4) Shooter/Flywheel/Voltage", getFlywheelVoltage().in(Volts));
        SmartDashboard.putNumber("4) Shooter/Flywheel/Current", getFlywheelCurrent().in(Amps));
        SmartDashboard.putBoolean("4) Shooter/Up to speed", isUpToSpeed.getAsBoolean());    

        // ##### Component Logs

        // SmartDashboard.putNumber("4) Shooter/Hood/Angle", getHoodAngle().in(Degrees));
        // SmartDashboard.putNumber("4) Shooter/Hood/Target Angle", targetAngle.in(Degrees));
        // SmartDashboard.putNumber("4) Shooter/Hood/RPM", getHoodVelocity().in(RPM));
        // // SmartDashboard.putNumber("4) Shooter/Hood/Wheel Radians",getAngularVelocity().in(RadiansPerSecond));
        // SmartDashboard.putNumber("4) Shooter/Hood/Voltage", getHoodVoltage().in(Volts));
        // SmartDashboard.putNumber("4) Shooter/Hood/Current", getHoodCurrent().in(Amps));
        // SmartDashboard.putBoolean("4) Shooter/Hood/At Angle", atAngleT().getAsBoolean());
        // SmartDashboard.putNumber("4) Shooter/Hood/Angle Tolerance", degreesTolerance.in(Degrees));
        

        // SmartDashboard.putNumber("4) Shooter/Flywheel/RPM", getFlywheelVelocity().in(RPM));
        // // SmartDashboard.putNumber("4) Shooter/Flywheel/Wheel Radians", getAngularVelocity().in(RadiansPerSecond));
        // SmartDashboard.putNumber("4) Shooter/Flywheel/Voltage", getFlywheelVoltage().in(Volts));
        // SmartDashboard.putNumber("4) Shooter/Flywheel/Target RPM", targetVelocity.in(RPM));
        // SmartDashboard.putNumber("4) Shooter/Flywheel/Current", getFlywheelCurrent().in(Amps));
        // SmartDashboard.putBoolean("4) Shooter/Flywheel/Up to speed", upToSpeedT().getAsBoolean());
        // SmartDashboard.putNumber("4) Shooter/Flywheel/Velocity Tolerance", RPMTolerance.in(RPM));
    }


    // SIMULATION
    SingleJointedArmSim hoodSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                    DCMotor.getKrakenX60(1),
                    kHoodMomentOfInertia.in(KilogramSquareMeters),
                    kHoodGearRatio),
            DCMotor.getKrakenX60(1),
            kHoodGearRatio,
            kHoodLength.in(Meters),
            kHoodMinAngle.in(Radians),
            kHoodMaxAngle.in(Radians),
            true, 
            kHoodMinAngle.in(Radians));

    FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(2),
                    kFlywheelMomentOfInertia.in(KilogramSquareMeters),
                    kFlywheelGearRatio),
            DCMotor.getKrakenX60(2),
            kFlywheelGearRatio);

    DCMotorSim fwMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(2),
                    kFlywheelMomentOfInertia.in(KilogramSquareMeters),
                    kFlywheelGearRatio),
            DCMotor.getKrakenX60(2));


    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = hMotor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;

        motorSimState.setSupplyVoltage(hMotor.getSupplyVoltage().getValue());// TODO: Add friction? Also, idk that the voltage should be accessed like this
        hoodSim.setInputVoltage(motorSimState.getMotorVoltage());

        hoodSim.update(0.02);

        motorSimState.setRawRotorPosition(Radians.of(hoodSim.getAngleRads() * kHoodGearRatio));
        motorSimState.setRotorVelocity(RadiansPerSecond.of(hoodSim.getVelocityRadPerSec() * kHoodGearRatio));


        TalonFXSimState fwMotorSimState = fwLeftMotor.getSimState();
        fwMotorSimState.Orientation = ChassisReference.Clockwise_Positive;// TODO: Fix, idk what it means

        fwMotorSimState.setSupplyVoltage(fwLeftMotor.getSupplyVoltage().getValue());// TODO: Add friction? Also, idk that
                                                                                // the voltage should be accessed like
                                                                                // this
        fwMotorSim.setInputVoltage(fwMotorSimState.getMotorVoltage());

        fwMotorSim.update(0.02);

        fwMotorSimState.setRawRotorPosition(fwMotorSim.getAngularPositionRotations() * kFlywheelGearRatio);
        fwMotorSimState.setRotorVelocity(fwMotorSim.getAngularVelocityRPM() / 60 * kFlywheelGearRatio);
        // shaft RPM --> rotations per second --> motor rotations per second
        double voltage = fwMotorSim.getInputVoltage();
        flywheelSim.setInput(voltage);
        flywheelSim.update(0.02);
    }

    public static class State implements Interpolatable<State>{
        private Angle angle; 
        private AngularVelocity velocity; 
        private Time tof;


        public State(Angle angle, AngularVelocity velocity, Time tof){
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
