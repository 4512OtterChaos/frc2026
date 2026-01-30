package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter.ShooterConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Climber.ClimberConstants.*;

    
public class Climber extends SubsystemBase {
    public TalonFX motor = new TalonFX(kMotorID);

    private Distance targetHeight = kMinHeight;

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);

    public Climber(){
        motor.getConfigurator().apply(kConfig);
        SmartDashboard.putData("Climber/Subsystem", this);
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
    
    public Distance getHeight(){
        return Inches.of(positionStatus.getValueAsDouble());
    }
    
    public Distance getTargetHeight() {
        return targetHeight;
    }

    public LinearVelocity getLinearVelocity(){
        return InchesPerSecond.of(velocityStatus.getValueAsDouble());// TODO: check if this works
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
    
    public void setAngle(Distance distance){
        motor.setControl(mmRequest.withPosition(angle));
        targetAngle = angle;
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    // public Command setDistanceC(Distance distance){
    //     return run(()-> setAngle(angle)).until(atAngleT()).withName("Set angle: " + angle);
    // }

    public Trigger atTargetHeight(){
        return new Trigger(getHeight() <= kD);
    }

    public void log(){
        SmartDashboard.putNumber("Shooter/Hood/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter/Hood/Current", getCurrent().in(Amps));
    }
}
