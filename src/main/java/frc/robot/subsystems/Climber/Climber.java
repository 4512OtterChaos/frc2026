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

    private Distance targetHeight = kMinHeight;

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);

    public Climber(){
        motor.getConfigurator().apply(kConfig);
        SmartDashboard.putData("Climber/Subsystem", this);
        resetHeight(kMinHeight);
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
        height = Inches.of(MathUtil.clamp(height.in(Inches), kMinHeight.in(Inches), kMaxHeight.in(Inches)));
        motor.setControl(mmRequest.withPosition(height.in(Inches)));
        targetHeight = height;
    }

    public boolean atTargetHeight(){
        return getHeight().in(Inches) - targetHeight.in(Inches) <= kHeightTolerance.in(Inches);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    public Command setHeightC(Distance height){
        return run(()-> setHeight(height)).until(atTargetHeightT()).withName("Set height: " + height);
    }

    public Command setMinHeightC(){
        return setHeightC(kMinHeight);
    }

    public Command setMaxHeight(){
        return setHeightC(kMaxHeight);
    }

    public Trigger atTargetHeightT(){
        return new Trigger(()-> atTargetHeight()).debounce(kDebounceTime);
    }

    public void log(){  
        SmartDashboard.putNumber("Climber/Height", getHeight().in(Inches));
        SmartDashboard.putNumber("Climber/Target Height", targetHeight.in(Inches));
        SmartDashboard.putNumber("Climber/Inches Per Second", getLinearVelocity().in(InchesPerSecond));
        SmartDashboard.putNumber("Climber/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Climber/Current", getCurrent().in(Amps));
        SmartDashboard.putBoolean("Climber/At Height", atTargetHeightT().getAsBoolean());
        SmartDashboard.putNumber("Climber/Height Tolerance", kHeightTolerance.in(Inches));
    }
}
/*
 * Possibilities:
 *    Stop
 *    Is manual (variable)
 *    Is stalled
 *    Tunable numbers
 */