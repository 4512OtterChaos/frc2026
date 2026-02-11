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
import static frc.robot.subsystems.Climber.ClimberConstants.*;

    
public class Climber extends SubsystemBase {
    public TalonFX motor = new TalonFX(kMotorID);

    private Distance targetHeight = Inches.of(minHeight.get());

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);

    public Climber(){
        motor.getConfigurator().apply(kConfig);
        SmartDashboard.putData("Climber/Subsystem", this);
        resetHeight(Inches.of(minHeight.get()));
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

    public void resetHeight(Distance height){
        motor.setPosition(height.in(Inches));
    }

    public void setVoltage(double voltage){
        motor.setVoltage(voltage);        
    }
    
    public void setHeight(Distance height){
        height = Inches.of(MathUtil.clamp(height.in(Inches), minHeight.get(), maxHeight.get()));
        motor.setControl(mmRequest.withPosition(height.in(Inches)));
        targetHeight = height;
    }

    public boolean atTargetHeight(){
        return getHeight().in(Inches) - targetHeight.in(Inches) <= heightTolerance.get();
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    public Command setHeightC(Distance height){
        return run(()-> setHeight(height)).until(atTargetHeightT()).withName("Set height: " + height);
    }

    public Command setMinHeightC(){
        return setHeightC(Inches.of(minHeight.get()));
    }

    public Command setMaxHeight(){
        return setHeightC(Inches.of(maxHeight.get()));
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
        SmartDashboard.putNumber("Climber/Height", getHeight().in(Inches));
        SmartDashboard.putNumber("Climber/Target Height", targetHeight.in(Inches));
        SmartDashboard.putNumber("Climber/Inches Per Second", getLinearVelocity().in(InchesPerSecond));
        SmartDashboard.putNumber("Climber/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Climber/Current", getCurrent().in(Amps));
        SmartDashboard.putBoolean("Climber/At Height", atTargetHeightT().getAsBoolean());
        SmartDashboard.putNumber("Climber/Height Tolerance", heightTolerance.get());
    }
}
/*
 * Possibilities:
 *    Stop
 *    Is manual (variable)
 *    Is stalled
 *    Tunable numbers: DONE
 */