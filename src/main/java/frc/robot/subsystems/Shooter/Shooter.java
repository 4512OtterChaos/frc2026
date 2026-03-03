package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.subsystems.Shooter.ShooterConstants.RPMTolerance;
import static frc.robot.subsystems.Shooter.ShooterConstants.degreesTolerance;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelDebounceTime;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelIdleRPM;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelkA;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelkD;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelkI;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelkP;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelkS;
import static frc.robot.subsystems.Shooter.ShooterConstants.flywheelkV;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodAcceleration;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodCruiseVelocity;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodDebounceTime;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodMaxAngle;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodMinAngle;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodkA;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodkD;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodkG;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodkI;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodkP;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodkS;
import static frc.robot.subsystems.Shooter.ShooterConstants.hoodkV;
import static frc.robot.subsystems.Shooter.ShooterConstants.kFlywheelConfig;
import static frc.robot.subsystems.Shooter.ShooterConstants.kFlywheelGearRatio;
import static frc.robot.subsystems.Shooter.ShooterConstants.kFlywheelMomentOfInertia;
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodConfig;
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodGearRatio;
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodLength;
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodMaxAngle;
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodMinAngle;
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodMomentOfInertia;
import static frc.robot.subsystems.Shooter.ShooterConstants.kHoodMotorID;
import static frc.robot.subsystems.Shooter.ShooterConstants.kLeftMotorID;
import static frc.robot.subsystems.Shooter.ShooterConstants.kRightMotorID;

public class Shooter extends SubsystemBase {
    private Hood hood = new Hood(); // TODO: delete these later
    private Flywheel flywheel = new Flywheel();

    public TalonFX fwLeftMotor = new TalonFX(kLeftMotorID);
    public TalonFX fwRightMotor = new TalonFX(kRightMotorID);

    public TalonFX hMotor = new TalonFX(kHoodMotorID);

    private AngularVelocity targetVelocity = RPM.of(0); // flywheel
    private Angle targetAngle = Degrees.of(hoodMinAngle.get()); //hood

    private final StatusSignal<Angle> fwPositionStatus = fwLeftMotor.getPosition();
    private final StatusSignal<AngularVelocity> fwVelocityStatus = fwLeftMotor.getVelocity();
    private final StatusSignal<Voltage> fwVoltageStatus = fwLeftMotor.getMotorVoltage();
    private final StatusSignal<Current> fwStatorStatus = fwLeftMotor.getStatorCurrent();

    private final StatusSignal<Angle> hPositionStatus = hMotor.getPosition();
    private final StatusSignal<AngularVelocity> hVelocityStatus = hMotor.getVelocity();
    private final StatusSignal<Voltage> hVoltageStatus = hMotor.getMotorVoltage();
    private final StatusSignal<Current> hStatorStatus = hMotor.getStatorCurrent();

    private final MotionMagicVoltage mmHoodRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final VelocityVoltage velocityrequest = new VelocityVoltage(0);

    public Shooter() {
        // FLYWHEEL
        fwLeftMotor.getConfigurator().apply(kFlywheelConfig);
        var fwRightConfig = kFlywheelConfig.clone();
        fwRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        fwRightMotor.getConfigurator().apply(fwRightConfig);
        // fwRightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // HOOD
        hMotor.getConfigurator().apply(kHoodConfig);
        resetAngle(Degrees.of(hoodMinAngle.get()));

        SmartDashboard.putData("Shooter/Subsystem", this);
    }

    @Override 
    public void periodic() {
        // HOOD PERIODIC
        BaseStatusSignal.refreshAll(
                hPositionStatus,
                hVelocityStatus,
                hVoltageStatus,
                hStatorStatus);
        hMotor.setControl(mmHoodRequest.withPosition(targetAngle));

        if (getHoodAngle().isNear(Degrees.of(targetAngle.in(Degrees)), Degrees.of(0.5))){
        //TODO: find a better way to do this
        setHoodVoltage(0);
        }

        // FLYWHEEL PERIODIC
        BaseStatusSignal.refreshAll(
                fwPositionStatus,
                fwVelocityStatus,
                fwVoltageStatus,
                fwStatorStatus);
        fwLeftMotor.setControl(velocityrequest.withVelocity(targetVelocity));
        fwRightMotor.setControl(velocityrequest.withVelocity(targetVelocity));

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
        angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), hoodMinAngle.get(), hoodMaxAngle.get()));
        targetAngle = angle;
    }

    public Command setAngleC(Angle angle) {
        return runOnce(() -> setAngle(angle)).until(atAngleT()).withName("Set angle: " + angle);
    }

    public Command setMinAngleC() {
        return setAngleC(Degrees.of(hoodMinAngle.get()));
    }

    public boolean atAngle() {
        return Math.abs(targetAngle.in(Degrees) - getHoodAngle().in(Degrees)) < degreesTolerance.get();
    }

    public Trigger atAngleT() {
        return new Trigger(() -> atAngle()).debounce(hoodDebounceTime.get());
    }

    // FLYWHEEL
    public Angle getFlywheelAngle() {
       return fwPositionStatus.getValue();
    }

    public AngularVelocity getFlywheelVelocity() {
        return fwVelocityStatus.getValue();
    }

    public Voltage getFlywheelVoltage() {
        return fwVoltageStatus.getValue();
    }

    public Current getFlywheelCurrent() {
        return fwStatorStatus.getValue();
    }

    public boolean upToSpeed() {
        return Math.abs(targetVelocity.in(RPM) - getFlywheelVelocity().in(RPM)) < RPMTolerance.get();
    }

    public Trigger upToSpeedT() {
        return new Trigger(() -> upToSpeed()).debounce(flywheelDebounceTime.get());
    }

    public AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    public Command setVelocityC(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity)).until(upToSpeedT()).withName("Set velocity: " + velocity);
    }

    public void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
    }
    
    //OVERALL
    public Command set(Angle angle, AngularVelocity velocity) {
        return parallel(setAngleC(angle), setVelocityC(velocity));
    }

    public void changeTunable() {
        hoodMinAngle.poll();
        hoodMaxAngle.poll();
        hoodDebounceTime.poll();
        degreesTolerance.poll();
        hoodkP.poll();
        hoodkI.poll();
        hoodkD.poll();
        hoodkG.poll();
        hoodkS.poll();
        hoodkV.poll();
        hoodkA.poll();
        hoodCruiseVelocity.poll();
        hoodAcceleration.poll();

        flywheelIdleRPM.poll();
        flywheelDebounceTime.poll();
        RPMTolerance.poll();
        flywheelkP.poll();
        flywheelkI.poll();
        flywheelkD.poll();
        flywheelkS.poll();
        flywheelkV.poll();
        flywheelkA.poll();

        int hash = hashCode();

        if (hoodkP.hasChanged(hash) || hoodkI.hasChanged(hash) || hoodkD.hasChanged(hash) || hoodkG.hasChanged(hash) || hoodkS.hasChanged(hash) || hoodkV.hasChanged(hash) || hoodkA.hasChanged(hash) || flywheelkP.hasChanged(hash) || flywheelkI.hasChanged(hash) || flywheelkD.hasChanged(hash) || flywheelkS.hasChanged(hash) || flywheelkV.hasChanged(hash) || flywheelkA.hasChanged(hash)) {
            kHoodConfig.Slot0.kP = hoodkP.get();
            kHoodConfig.Slot0.kI = hoodkI.get();
            kHoodConfig.Slot0.kD = hoodkD.get();
            kHoodConfig.Slot0.kG = hoodkG.get();
            kHoodConfig.Slot0.kS = hoodkS.get();
            kHoodConfig.Slot0.kV = hoodkV.get();
            kHoodConfig.Slot0.kA = hoodkA.get();
            hMotor.getConfigurator().apply(kHoodConfig.Slot0);

            kFlywheelConfig.Slot0.kP = flywheelkP.get();
            kFlywheelConfig.Slot0.kI = flywheelkI.get();
            kFlywheelConfig.Slot0.kD = flywheelkD.get();
            kFlywheelConfig.Slot0.kS = flywheelkS.get();
            kFlywheelConfig.Slot0.kV = flywheelkV.get();
            kFlywheelConfig.Slot0.kA = flywheelkA.get();
            fwLeftMotor.getConfigurator().apply(kFlywheelConfig.Slot0);
        }

        if (hoodCruiseVelocity.hasChanged(hash) || hoodAcceleration.hasChanged(hash)) {
            kHoodConfig.MotionMagic.MotionMagicCruiseVelocity = hoodCruiseVelocity.get();
            kHoodConfig.MotionMagic.MotionMagicAcceleration = hoodAcceleration.get();
            hMotor.getConfigurator().apply(kHoodConfig.MotionMagic);
        }

    }

    public void log() {
        SmartDashboard.putNumber("Shooter/Hood/Angle", getHoodAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter/Hood/Target Angle", targetAngle.in(Degrees));
        SmartDashboard.putNumber("Shooter/Hood/RPM", getFlywheelVelocity().in(RPM));
        // SmartDashboard.putNumber("Shooter/Hood/Wheel Radians",getAngularVelocity().in(RadiansPerSecond));
        SmartDashboard.putNumber("Shooter/Hood/Voltage", getHoodVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter/Hood/Current", getHoodCurrent().in(Amps));
        SmartDashboard.putBoolean("Shooter/Hood/At Angle", atAngleT().getAsBoolean());
        SmartDashboard.putNumber("Shooter/Hood/Angle Tolerance", degreesTolerance.get());

        SmartDashboard.putNumber("Shooter/Flywheel/RPM", getFlywheelVelocity().in(RPM));
        // SmartDashboard.putNumber("Shooter/Flywheel/Wheel Radians", getAngularVelocity().in(RadiansPerSecond));
        SmartDashboard.putNumber("Shooter/Flywheel/Voltage", getFlywheelVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter/Flywheel/Target RPM", targetVelocity.in(RPM));
        SmartDashboard.putNumber("Shooter/Flywheel/Current", getFlywheelCurrent().in(Amps));
        SmartDashboard.putBoolean("Shooter/Flywheel/Up to speed", upToSpeedT().getAsBoolean());
        SmartDashboard.putNumber("Shooter/Flywheel/Velocity Tolerance", RPMTolerance.get());
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
            true, // TODO:Include gravity?
            kHoodMinAngle.in(Radians));

    DCMotorSim hMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    kHoodMomentOfInertia.in(KilogramSquareMeters),
                    kHoodGearRatio),
            DCMotor.getKrakenX60(1));


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
        TalonFXSimState hMotorSimState = hMotor.getSimState();
        hMotorSimState.Orientation = ChassisReference.Clockwise_Positive;

        hMotorSimState.setSupplyVoltage(hMotor.getSupplyVoltage().getValue());
        hMotorSim.setInputVoltage(hMotorSimState.getMotorVoltage());

        hMotorSim.update(0.02);

        hMotorSimState.setRawRotorPosition(hMotorSim.getAngularPositionRotations() * kHoodGearRatio);
        hMotorSimState.setRotorVelocity(hMotorSim.getAngularVelocityRPM() / 60 * kHoodGearRatio);
        // shaft RPM --> rotations per second --> motor rotations per second
        double hoodVoltage = hMotorSim.getInputVoltage();
        hoodSim.setInput(hoodVoltage);
        hoodSim.update(0.02);


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
}
