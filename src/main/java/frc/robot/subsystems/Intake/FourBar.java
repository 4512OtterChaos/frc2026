package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.OCTrigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FourBar extends SubsystemBase {
    private TalonFX motor = new TalonFX(kFourBarMotorID);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private Current targetCurrent = Amps.of(0);
    private TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);

    /** Velocity is near 0 */
    public final Trigger isStationary = new Trigger(() -> getVelocity().isNear(DegreesPerSecond.of(0), DegreesPerSecond.of(3)));
    /** Current is above stall threshold */
    public final Trigger isStalled = new Trigger(() -> getCurrent().gt(stallThreshold.get()));
    /** Stationary and pulling current above threshold for some time */
    public final Trigger isHomed = OCTrigger.debounce(isStalled.and(isStationary), () -> stallTime.in(Seconds));


    private boolean readyToOscillate = false;
    private boolean doneOscillating = false;
    private boolean stayRetracted = true;

    public FourBar() {
        motor.getConfigurator().apply(kFourBarConfig);
        
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        statorStatus.setUpdateFrequency(50);

        ParentDevice.optimizeBusUtilizationForAll(motor);

        SmartDashboard.putData("2) Intake/Four Bar/Subsystem", this);

        resetAngle(kFourBarMaxAngle);
        if (Utils.isSimulation()) {
            resetAngle(Degrees.of(0));
        }
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                positionStatus,
                velocityStatus,
                voltageStatus,
                statorStatus);
        changeTunable();

        motor.setControl(torqueRequest.withOutput(targetCurrent));
        log();
    }

    public Angle getAngle() {
        return positionStatus.getValue();
    }

    public AngularVelocity getVelocity() {
        return velocityStatus.getValue();
    }

    public Voltage getVoltage() {
        return voltageStatus.getValue();
    }

    public Current getCurrent() {
        return statorStatus.getValue();
    }

    public void resetAngle(Angle angle) {
        motor.setPosition(angle);
    }

    public void setCurrent(Current current) {
        targetCurrent = current;
        // controlMode = ControlMode.Torque;
    }

    public Command setCurrentC(Current current) {
        return run(()-> setCurrent(current)).withName("Set Current: " + current.in(Amps));
    }

    public Command setRetractCurrent1C() {
        return run(()->setCurrent(retractCurrent1.get()));
    }

    public Command setRetractCurrent2C() {
        return run(()->setCurrent(retractCurrent2.get()));
    }

    public Command retractC() {
        return sequence(
            setRetractCurrent1C().withTimeout(0.4),
            setRetractCurrent2C().withTimeout(0.5)
        ).withName("Retract Fourbar");
    }

    public Command setExtendCurrent1C() {
        return run(()->{
            stayRetracted = false;
            setCurrent(extendCurrent1.get());
        });
    }

    public Command setExtendCurrent2C() {
        return run(()->{
            stayRetracted = false;
            setCurrent(extendCurrent2.get());
        });
    }

    public Command extendC() {
        return sequence(
            setExtendCurrent1C().withTimeout(0.35),
            setExtendCurrent2C().withTimeout(0.5)
        ).finallyDo(()-> resetDoneOscillating()).withName("Extend Fourbar");
    }

    public Command retractPermanantC() {
        return sequence(
            runOnce(()-> stayRetracted = true),
            retractC()
        ).withName("Retract Fourbar");
    }

    public Command stayExtendedC(){
        return Commands.either(
            extendC(),
            runOnce(()->setCurrent(Amps.of(0))),
            ()-> (! (stayRetracted || readyToOscillate)) &&  !(getAngle().isNear(kFourBarMinAngle, fourBarAngleTolerance.get()))
        ).repeatedly();
    }

    public Command oscillateC() {
        return repeatingSequence(
            retractC().until(isHomed),
            extendC().until(isHomed)
        ).finallyDo(()-> doneOscillating = true).withName("Otterscillate");
    }

    public void setReadyToOscillate(boolean isTrue) {
        readyToOscillate = isTrue;
    }

    public Command setReadyToOscillateC(boolean isTrue) {
        return Commands.runOnce(()-> setReadyToOscillate(isTrue));
    }

    public Trigger readyToOscillateT() {
        return new Trigger(()-> readyToOscillate);
    }

    public Trigger doneOscillatingT() {
        return new Trigger(() ->doneOscillating);
    }

    public void resetDoneOscillating() {
        doneOscillating = false;
    }

    public void changeTunable() {
        // fourBarMinAngle.poll();
        // fourBarMaxAngle.poll();
        // angleTolerance.poll();
        retractCurrent1.poll();
        retractCurrent2.poll();
        extendCurrent1.poll();
        extendCurrent2.poll();
        
    }

    public void log() {
        // SmartDashboard.putNumber("2) Intake/Four Bar/Angle Degrees", getAngle().in(Degrees));
        // SmartDashboard.putNumber("2) Intake/Four Bar/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("2) Intake/Four Bar/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("2) Intake/Four Bar/Current", getCurrent().in(Amps));
        SmartDashboard.putNumber("2) Intake/Four Bar/Target Current", targetCurrent.in(Amps));

        SmartDashboard.putBoolean("test", doneOscillating);
        SmartDashboard.putBoolean("2) Intake/Four Bar/Is Stationary", isStationary.getAsBoolean());
        SmartDashboard.putBoolean("2) Intake/Four Bar/Is Stalled", isStalled.getAsBoolean());
        SmartDashboard.putBoolean("2) Intake/Four Bar/Is Homed", isHomed.getAsBoolean());
    }

    // Simulation
    SingleJointedArmSim fourBarSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        kFourBarGearRatio,
        kFourBarMomentOfInertia.in(KilogramSquareMeters),
        kFourBarArmLength.in(Meters),
        kFourBarMinAngle.in(Radians),
        kFourBarMaxAngle.in(Radians),
        true,
        kFourBarMaxAngle.in(Radians)
    );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;

        motorSimState.setSupplyVoltage(motor.getSupplyVoltage().getValue());
        fourBarSim.setInputVoltage(motorSimState.getMotorVoltage());

        fourBarSim.update(0.02);

        motorSimState.setRawRotorPosition(Radians.of(fourBarSim.getAngleRads() * kFourBarGearRatio));
        motorSimState.setRotorVelocity(RadiansPerSecond.of(fourBarSim.getVelocityRadPerSec() * kFourBarGearRatio));
    }
}