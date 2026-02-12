package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.Climber.ClimberConstants.*;

public class Climber extends SubsystemBase {

    private final TalonFX motor = new TalonFX(kMotorID);

    // Store target in ROTATIONS
    private Angle targetRotations = Rotations.of(0);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private final MotionMagicVoltage mmRequest =
            new MotionMagicVoltage(0).withEnableFOC(false);

    private final Trigger atTargetTrigger =
            new Trigger(this::atTargetHeight);

    public Climber() {
        motor.getConfigurator().apply(kConfig);
        SmartDashboard.putData("Climber/Subsystem", this);
        resetHeight(Rotations.of(minHeight.get()));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                positionStatus,
                velocityStatus,
                voltageStatus,
                statorStatus
        );

        changeTunable();
        log();
    }

    /* ------------------- Getters ------------------- */

    public Angle getRotations() {
        return Rotations.of(positionStatus.getValueAsDouble());
    }

    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(velocityStatus.getValueAsDouble());
    }

    public Voltage getVoltage() {
        return voltageStatus.getValue();
    }

    public Current getCurrent() {
        return statorStatus.getValue();
    }

    /* ------------------- Control ------------------- */

    public void resetHeight(Angle rotations) {
        motor.setPosition(rotations.in(Rotations));
        targetRotations = rotations;
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    public void setHeight(Angle rotations) {
        rotations = Rotations.of(
                MathUtil.clamp(
                        rotations.in(Rotations),
                        minHeight.get(),
                        maxHeight.get()
                )
        );

        targetRotations = rotations;
        motor.setControl(mmRequest.withPosition(rotations.in(Rotations)));
    }

    public boolean atTargetHeight() {
        return Math.abs(
                getRotations().in(Rotations)
                        - targetRotations.in(Rotations)
        ) <= heightTolerance.get();
    }

    /* ------------------- Commands ------------------- */

    public Command setHeightC(Angle rotations) {
        return runOnce(() -> setHeight(rotations))
                .until(atTargetTrigger)
                .withName("Set height: " + rotations);
    }

    public Command setMinHeightC() {
        return setHeightC(Rotations.of(minHeight.get()));
    }

    public Command setMaxHeightC() {
        return setHeightC(Rotations.of(maxHeight.get()));
    }

    public Command climbC() {
        return sequence(
                setMaxHeightC(),
                waitUntil(atTargetTrigger),
                setMinHeightC()
        ).withName("Climb");
    }

    /* ------------------- Tunables ------------------- */

    public void changeTunable() {
        maxHeight.poll();
        minHeight.poll();
        heightTolerance.poll();
        debounceTime.poll();
        kP.poll();
        kI.poll();
        kD.poll();
        kS.poll();
        kV.poll();
        kA.poll();

        int hash = hashCode();

        if (kP.hasChanged(hash) || kI.hasChanged(hash) ||
            kD.hasChanged(hash) || kS.hasChanged(hash) ||
            kV.hasChanged(hash) || kA.hasChanged(hash)) {

            kConfig.Slot0.kP = kP.get();
            kConfig.Slot0.kI = kI.get();
            kConfig.Slot0.kD = kD.get();
            kConfig.Slot0.kS = kS.get();
            kConfig.Slot0.kV = kV.get();
            kConfig.Slot0.kA = kA.get();

            motor.getConfigurator().apply(kConfig.Slot0);
        }
    }

    /* ------------------- Logging ------------------- */

    public void log() {
        SmartDashboard.putNumber("Climber/Rotations",
                getRotations().in(Rotations));
        SmartDashboard.putNumber("Climber/Target Rotations",
                targetRotations.in(Rotations));
        SmartDashboard.putNumber("Climber/Velocity (rps)",
                getVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Climber/Voltage",
                getVoltage().in(Volts));
        SmartDashboard.putNumber("Climber/Current",
                getCurrent().in(Amps));
    }
}