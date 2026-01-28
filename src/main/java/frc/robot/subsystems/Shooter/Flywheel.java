package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Shooter.ShooterConstants.kFlywheelConfig;
import static frc.robot.subsystems.Shooter.ShooterConstants.kDebounceTime;
import static frc.robot.subsystems.Shooter.ShooterConstants.kLeftMotorID;
import static frc.robot.subsystems.Shooter.ShooterConstants.kRightMotorID;
import static frc.robot.subsystems.Shooter.ShooterConstants.kVelocityTolerance;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Flywheel extends SubsystemBase {
    public TalonFX leftMotor = new TalonFX(kLeftMotorID);
    public TalonFX rightMotor = new TalonFX(kRightMotorID);

    private AngularVelocity targetVelocity = RPM.of(0);

    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();

    private final VelocityDutyCycle velocityrequest = new VelocityDutyCycle(0);
    

    public Flywheel() {
        leftMotor.getConfigurator().apply(kFlywheelConfig);
        rightMotor.getConfigurator().apply(kFlywheelConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        SmartDashboard.putData("Shooter/Flywheel/Subsystem", this);
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(
            positionStatus,
            velocityStatus,
            voltageStatus,
            statorStatus
        );
        
        log();
    }

    public Angle getAngle(){
        return positionStatus.getValue();
    }

    public AngularVelocity getAngularVelocity(){
        return velocityStatus.getValue();
    }

    public AngularVelocity getTargetVelocity(){
        return targetVelocity;
    }

    public Voltage getVoltage(){
        return voltageStatus.getValue();
    }

    public Current getCurrent(){
        return statorStatus.getValue();
    }

    public void setVoltage(double voltage){
        leftMotor.setVoltage(voltage);        
    }

    public void setVelocity(AngularVelocity velocity){
        leftMotor.setControl(velocityrequest.withVelocity(velocity));
        targetVelocity = velocity;
    }

    public boolean upToSpeed(){
        return Math.abs(targetVelocity.in(RPM) - getAngularVelocity().in(RPM)) < kVelocityTolerance.in(RPM);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    public Command setVelocityC(AngularVelocity velocity){
        return runOnce(()-> setVelocity(velocity)).until(upToSpeedT()).withName("Set velocity: " + velocity);
    }

    public Trigger upToSpeedT(){
        return new Trigger(()-> upToSpeed()).debounce(kDebounceTime);
    }

    public void log(){
        SmartDashboard.putNumber("Shooter/Flywheel/Shooter Wheel Angle", getAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter/Flywheel/Wheel RPM", getAngularVelocity().in(RPM));
        // SmartDashboard.putNumber("Shooter/Flywheel/Wheel Radians", getAngularVelocity().in(RadiansPerSecond));
        SmartDashboard.putNumber("Shooter/Flywheel/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter/Flywheel/Target RPM", targetVelocity.in(RPM));
        SmartDashboard.putNumber("Shooter/Flywheel/Current", getCurrent().in(Amps));
        SmartDashboard.putBoolean("Shooter/Flywheel/Up to speed", upToSpeedT().getAsBoolean());
        SmartDashboard.putNumber("Shooter/Flywheel/Velocity Tolerance", ShooterConstants.kVelocityTolerance.in(RPM));
    }
}
