package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;


public class Hood extends SubsystemBase {

    public TalonFX motor = new TalonFX(kHoodMotorID);

    private Angle targetAngle = Degrees.of(0);
 
    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);   
    
    public Hood(){
        motor.getConfigurator().apply(kHoodConfig);
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
    }
    
    public Angle getAngle(){
        return positionStatus.getValue();
    }

    public Angle getTargetAngle(){
        return targetAngle;
    }

    public AngularVelocity getAngularVelocity(){
        return velocityStatus.getValue();
    }

    public Voltage getVoltage(){
        return voltageStatus.getValue();
    }

    public Current getCurrent(){
        return statorStatus.getValue();
    }

    public void setVoltage(double voltage){
        motor.setVoltage(voltage);        
    }

    public void setAngle(Angle angle){
        motor.setControl(mmRequest.withPosition(angle));
        targetAngle = angle;
    }
    
    public boolean atAngle(){
        return Math.abs(targetAngle.in(Degrees) - getAngle().in(Degrees)) < kAngleTolerance.in(Degrees);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    public Command setAngleC(Angle angle){
        return run(()-> setAngle(angle)).until(atAngleT()).withName("Set angle: " + angle);
    }

    public Trigger atAngleT(){
        return new Trigger(()-> atAngle()).debounce(kDebounceTime);
    }

    public void log(){
        SmartDashboard.putNumber("Shooter/Hood/Angle", getAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter/Hood/Target Angle", targetAngle.in(Degrees));
        SmartDashboard.putNumber("Shooter/Hood/RPM", getAngularVelocity().in(RPM));
        // SmartDashboard.putNumber("Shooter/Hood/Wheel Radians", getAngularVelocity().in(RadiansPerSecond));
        SmartDashboard.putNumber("Shooter/Hood/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter/Hood/Current", getCurrent().in(Amps));
        SmartDashboard.putBoolean("Shooter/Hood/At Angle", atAngleT().getAsBoolean());
        SmartDashboard.putNumber("Shooter/Hood/Angle Tolerance", ShooterConstants.kAngleTolerance.in(Degrees));
    }
}