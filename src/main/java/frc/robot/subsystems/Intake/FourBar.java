package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
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
    
    public FourBar(){
        motor.getConfigurator().apply(kFourBarConfig);
        SmartDashboard.putData("Intake/Four Bar/Subsystem", this);
        resetAngle(Degrees.of(fourBarMaxDegrees.get()));
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

    public AngularVelocity getVelocity(){
        return velocityStatus.getValue();
    }

    public Voltage getVoltage(){
        return voltageStatus.getValue();
    }

    public Current getCurrent(){
        return statorStatus.getValue();
    }

    public void resetAngle(Angle angle){
        motor.setPosition(angle);
    }

    public void setVoltage(double voltage){
        if (getAngle().in(Degrees) >= fourBarMaxDegrees.get()){
            voltage = MathUtil.clamp(voltage, -12, 0);
        }
        if (getAngle().in(Degrees) <= fourBarMinDegrees.get()){
            voltage = MathUtil.clamp(voltage, 0, 12);
        }
        motor.setVoltage(voltage);
    }  

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);    
    }

    public Command setVoltageInC(){
        return setVoltageC(fourBarVoltageIn.get()).withName("Voltage In");
    }

    public Command setVoltageOutC(){
        return setVoltageC(fourBarVoltageOut.get()).withName("Voltage Out");
    }

    public Command lower(){
        return sequence(
            setVoltageC(fourBarVoltageOut.get()),
            waitUntil(atAngle(Degrees.of(fourBarMinDegrees.get()))),
            setVoltageC(0)
        );
    }

    public Trigger atAngle(Angle angle){
        return new Trigger(()-> (getAngle().in(Degrees) - angle.in(Degrees)) <= degreeTolerance.get());
    }

    public void changeTunable(){
        fourBarVoltageIn.poll();
        fourBarVoltageOut.poll();
        fourBarMinDegrees.poll();
        fourBarMaxDegrees.poll();
        degreeTolerance.poll();
    }

    public void log(){
        SmartDashboard.putNumber("Intake/Four Bar/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Intake/Four Bar/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Intake/Four Bar/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Intake/Four Bar/Current", getCurrent().in(Amps));
    }
}
