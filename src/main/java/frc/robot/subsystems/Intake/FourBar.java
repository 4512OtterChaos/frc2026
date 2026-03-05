package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FourBar extends SubsystemBase {
    private TalonFX motor = new TalonFX(kFourBarMotorID);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private Current targetCurrent = Amps.of(ampsIn.get());
    private TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);

    private Angle targetAngle = Degrees.of(fourBarMaxDegrees.get());
    private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    
    private Voltage targetVoltage = Volts.of(0);

    private ControlMode controlMode = ControlMode.Torque;

    public FourBar() {
        motor.getConfigurator().apply(kFourBarConfig);
        SmartDashboard.putData("Intake/Four Bar/Subsystem", this);
        resetAngle(Degrees.of(fourBarMaxDegrees.get()));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                positionStatus,
                velocityStatus,
                voltageStatus,
                statorStatus);
        changeTunable();

        switch (controlMode) {
            case Torque: 
                motor.setControl(torqueRequest.withOutput(targetCurrent));
                break;
            case MotionMagic:
                motor.setControl(mmRequest.withPosition(targetAngle));
                if (getAngle().isNear(Degrees.of(targetAngle.in(Degrees)), Degrees.of(0.2))) { //TODO: find a better way to do this
                    setVoltage(0);
                }
                break;
            case Voltage:
                motor.setVoltage(targetVoltage.in(Volts));
                break;
        }
        log();
    }

    public Angle getAngle() {
        return positionStatus.getValue();
    }

    public Angle getTargetAngle() {
        return targetAngle;
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
        controlMode = ControlMode.Torque;
    }

    public Command setCurrentC(Current current) {
        return run(()-> setCurrent(current));
    }

    public Command setCurrentInC() {
        return setCurrentC(Amps.of(ampsIn.get()));
    }

    public Command setCurrentOutC() {
        return setCurrentC(Amps.of(ampsOut.get()));
    }

    public Command oscillateC() {
        return sequence(
            setCurrentInC().withTimeout(1),
            setCurrentOutC().withTimeout(1)
        ).repeatedly().finallyDo(()-> setCurrent(Amps.of(ampsOut.get())));
    }

    public void setAngle(Angle angle) {
        targetAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), fourBarMinDegrees.get(), fourBarMaxDegrees.get()));
        controlMode = ControlMode.MotionMagic;
    }

    public Command setAngleC(Angle angle) {
        return run(() -> setAngle(angle));
    }

    public Command setMinAngleC() {
        return setAngleC(Degrees.of(fourBarMinDegrees.get()));
    }

    public Command setMaxAngleC() {
        return setAngleC(Degrees.of(fourBarMaxDegrees.get()));
    }

    public boolean atAngle(Angle angle) {
        return getAngle().isNear(angle, Degrees.of(degreeTolerance.get()));
    }

    public Trigger atAngleT(Angle angle) {
        return new Trigger(() -> atAngle(angle));
    }

    public void setVoltage(double voltage) {
        if (getAngle().in(Degrees) >= fourBarMaxDegrees.get()) {
            voltage = MathUtil.clamp(voltage, -12, 0);
        } else if (getAngle().in(Degree) <= fourBarMinDegrees.get()) {
            voltage = MathUtil.clamp(voltage, 0, 12);
        }
        targetVoltage = Volts.of(voltage);
        controlMode = ControlMode.Voltage;
    }

    public Command setVoltageC(double voltage) {
        return runOnce(() -> setVoltage(voltage)).withName("Set Voltage: " + voltage);
    }

    public Command setVoltageInC() {
        return setVoltageC(fourBarVoltageIn.get()).withName("Voltage In");
    }

    public Command setVoltageOutC() {
        return setVoltageC(fourBarVoltageOut.get()).withName("Voltage Out");
    }

    public void changeTunable() {
        fourBarVoltageIn.poll();
        fourBarVoltageOut.poll();
        fourBarMinDegrees.poll();
        fourBarMaxDegrees.poll();
        degreeTolerance.poll();
        ampsIn.poll();
        ampsOut.poll();
    }

    public void log() {
        SmartDashboard.putNumber("Intake/Four Bar/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Intake/Four Bar/Target Angle Degrees", targetAngle.in(Degrees));
        SmartDashboard.putNumber("Intake/Four Bar/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Intake/Four Bar/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Intake/Four Bar/Current", getCurrent().in(Amps));
    }

    // Simulation
    SingleJointedArmSim fourBarSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            kFourBarGearRatio,
            kFourBarMomentOfInertia.in(KilogramSquareMeters),
            kFourBarArmLength.in(Meters),
            fourBarMinDegrees.get(),
            fourBarMaxDegrees.get(),
            true,
            fourBarMinDegrees.get());

    DCMotorSim motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(2),
                    kFourBarMomentOfInertia.in(KilogramSquareMeters),
                    kFourBarGearRatio),
            DCMotor.getKrakenX60(2));

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;

        motorSimState.setSupplyVoltage(motor.getSupplyVoltage().getValue());// TODO: Add friction? Also, idk that the
                                                                            // voltage should be accessed like this
        motorSim.setInputVoltage(motorSimState.getMotorVoltage());

        motorSim.update(0.02);

        motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * kFourBarGearRatio);
        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60 * kFourBarGearRatio);
        double voltage = motorSim.getInputVoltage();
        fourBarSim.setInput(voltage);
        fourBarSim.update(0.02);
    }

    private enum ControlMode {
        Torque,
        MotionMagic,
        Voltage
    }
}