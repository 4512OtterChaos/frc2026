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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FourBar extends SubsystemBase {
    private TalonFX motor = new TalonFX(kFourBarMotorID);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private Current targetCurrent = Amps.of(0);
    private TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);

    private boolean readyToOscillate = false;
    private boolean doneOscillating = false;

    // private Angle targetAngle = fourBarMaxAngle.get();
    // private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    // private Voltage targetVoltage = Volts.of(0);

    // private ControlMode controlMode = ControlMode.Torque;

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

        // switch (controlMode) {
        //     case Torque:
        //         motor.setControl(torqueRequest.withOutput(targetCurrent));
        //         break;
        //     case MotionMagic:
        //         motor.setControl(mmRequest.withPosition(targetAngle));
        //         break;
        //     case Voltage:
        //         motor.setVoltage(targetVoltage.in(Volts));
        //         break;
        // }
        motor.setControl(torqueRequest.withOutput(targetCurrent));
        log();
    }

    public Angle getAngle() {
        return positionStatus.getValue();
    }

    // public Angle getTargetAngle() {
    //     return targetAngle;
    // }

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
            setRetractCurrent1C().withTimeout(0.2),
            setRetractCurrent2C()
        ).withTimeout(1.25).withName("Retract Fourbar");
    }

    public Command setExtendCurrent1C() {
        return run(()->setCurrent(extendCurrent1.get()));
    }

    public Command setExtendCurrent2C() {
        return run(()->setCurrent(extendCurrent2.get()));
    }

    public Command extendC() {
        return sequence(
            setExtendCurrent1C().withTimeout(0.2),
            setExtendCurrent2C()
        ).withTimeout(1).finallyDo(()-> resetDoneOscillating()).withName("Extend Fourbar");
    }

    public Command oscillateC() {
        return repeatingSequence(
            retractC().withTimeout(0.3),
            extendC().withTimeout(0.3)
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

    // public void setAngle(Angle angle) {
    //     targetAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), fourBarMinAngle.in(Degrees), fourBarMaxAngle.in(Degrees)));
    //     controlMode = ControlMode.MotionMagic;
    // }

    // public Command setAngleC(Angle angle) {
    //     return run(() -> setAngle(angle));
    // }

    // public Command setMinAngleC() {
    //     return setAngleC(fourBarMinAngle.get());
    // }

    // public Command setMaxAngleC() {
    //     return setAngleC(fourBarMaxAngle.get());
    // }

    // public boolean atAngle(Angle angle) {
    //     return getAngle().isNear(angle, angleTolerance.get());
    // }

    // public Trigger atAngleT(Angle angle) {
    //     return new Trigger(() -> atAngle(angle));
    // }

    // public void setVoltage(double voltage) {
    //     // if (getAngle().in(Degrees) >= fourBarMaxDegrees.get()) { //TODO: Re-enable?
    //     //     voltage = MathUtil.clamp(voltage, -12, 0);
    //     // } else if (getAngle().in(Degree) <= fourBarMinDegrees.get()) {
    //     //     voltage = MathUtil.clamp(voltage, 0, 12);
    //     // }
    //     targetVoltage = Volts.of(voltage);
    //     controlMode = ControlMode.Voltage;
    // }

    // public Command setVoltageC(double voltage) {
    //     return runOnce(() -> setVoltage(voltage)).withName("Set Voltage: " + voltage);
    // }

    // public Command setVoltageInC() {
    //     return setVoltageC(fourBarVoltageIn.get()).withName("Voltage In");
    // }

    // public Command setVoltageOutC() {
    //     return setVoltageC(fourBarVoltageOut.get()).withName("Voltage Out");
    // }

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
        // SmartDashboard.putNumber("2) Intake/Four Bar/Target Angle Degrees", targetAngle.in(Degrees));
        // SmartDashboard.putNumber("2) Intake/Four Bar/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("2) Intake/Four Bar/Voltage", getVoltage().in(Volts));
        // SmartDashboard.putNumber("2) Intake/Four Bar/Target Voltage", targetVoltage.in(Volts));
        SmartDashboard.putNumber("2) Intake/Four Bar/Current", getCurrent().in(Amps));
        SmartDashboard.putNumber("2) Intake/Four Bar/Target Current", targetCurrent.in(Amps));
        // SmartDashboard.putString("2) Intake/Four Bar/Control Mode", controlMode.toString());

        SmartDashboard.putBoolean("test", doneOscillating);
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

        motorSimState.setSupplyVoltage(motor.getSupplyVoltage().getValue());// TODO: Add friction? Also, idk that the voltage should be accessed like this
        fourBarSim.setInputVoltage(motorSimState.getMotorVoltage());

        fourBarSim.update(0.02);

        motorSimState.setRawRotorPosition(Radians.of(fourBarSim.getAngleRads() * kFourBarGearRatio));
        motorSimState.setRotorVelocity(RadiansPerSecond.of(fourBarSim.getVelocityRadPerSec() * kFourBarGearRatio));
    }

    // private enum ControlMode {
    //     Torque,
    //     MotionMagic,
    //     Voltage
    // }
}