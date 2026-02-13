package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.subsystems.Climber.ClimberConstants.*;

    
public class Climber extends SubsystemBase {
    public TalonFX motor = new TalonFX(kMotorID);

    private Angle targetRotationDegrees = Degrees.of(30000);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);

    public Climber(){
        motor.getConfigurator().apply(kConfig);
        SmartDashboard.putData("Climber/Subsystem", this);
        resetHeight(Degrees.of(2700));
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(
            positionStatus,
            velocityStatus,
            voltageStatus,
            statorStatus
        );
        changeTunable();
        log();
        motor.setControl(mmRequest.withPosition(targetRotationDegrees.in(Degrees)));
    }
    
    public Angle getHeight(){
        return Degrees.of(positionStatus.getValueAsDouble());
    }
    
    public Angle getTargetRotations() {
        return targetRotationDegrees.div(360);
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

    public void resetHeight(Angle height){
        motor.setPosition(height.in(Degrees));
    }

    public void setVoltage(double voltage){
        motor.setVoltage(voltage);        
    }
    
    public void setHeight(Angle height){
        height = Degrees.of(MathUtil.clamp(height.in(Degrees), minHeight.get(), maxHeight.get()));
        targetRotationDegrees = height;
    }

    public boolean atTargetHeight(){
        return getHeight().in(Degrees) - targetRotationDegrees.in(Degrees) <= heightTolerance.get();
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    public Command setHeightC(Angle angle){
        return run(()-> setHeight(angle)).until(atTargetHeightT()).withName("Set height: " + angle);
    }

    public Command setMinHeightC(){
        return setHeightC(Degrees.of(minHeight.get()));
    }

    public Command setMaxHeight(){
        return setHeightC(Degrees.of(maxHeight.get()));
    }

    public Command climbC(){
        return sequence(setMaxHeight(), waitUntil(atTargetHeightT()), setMinHeightC());
    }


    public Trigger atTargetHeightT(){
        return new Trigger(()-> atTargetHeight()).debounce(debounceTime.get());
    }

    public void changeTunable(){
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
     
        if (kP.hasChanged(hash) || kI.hasChanged(hash) || kD.hasChanged(hash) || kS.hasChanged(hash) || kV.hasChanged(hash) || kA.hasChanged(hash)) {
            kConfig.Slot0.kP = kP.get();
            kConfig.Slot0.kI = kI.get();
            kConfig.Slot0.kD = kD.get(); 
            kConfig.Slot0.kS = kS.get();
            kConfig.Slot0.kV = kV.get();
            kConfig.Slot0.kA = kA.get(); 
            motor.getConfigurator().apply(kConfig.Slot0);
        }   
    }


    public void log(){  
        SmartDashboard.putNumber("Climber/Height Rotation", getHeight().in(Degrees));
        SmartDashboard.putNumber("Climber/Target Height Rotation", targetRotationDegrees.in(Degrees));
        SmartDashboard.putNumber("Climber/Rotatationd Per Second", getLinearVelocity().in(InchesPerSecond));
        SmartDashboard.putNumber("Climber/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Climber/Current", getCurrent().in(Amps));
        SmartDashboard.putBoolean("Climber/At Height?", atTargetHeightT().getAsBoolean());
        SmartDashboard.putNumber("Climber/Height Tolerance Rotation", heightTolerance.get());
    }
}
/*
 * Possibilities:
 *    Stop
 *    Is manual (variable)
 *    Is stalled
 *    Tunable numbers: DONE
*/