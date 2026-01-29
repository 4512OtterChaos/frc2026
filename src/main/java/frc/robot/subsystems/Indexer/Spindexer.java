package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private TalonFX motor = new TalonFX(kSpindexerID);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    public Spindexer(){
        motor.getConfigurator().apply(kSpindexerConfig);
        SmartDashboard.putData("Indexer/Spindexer/Subsystem", this);
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(positionStatus, velocityStatus, voltageStatus, statorStatus);
        log();
    }

    public Angle getAngle() {
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

    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);    
    }

    public Command spindexC(){
        return setVoltageC(kSpindexerVoltage).withName("Spindex");
    }

    public void log(){
        SmartDashboard.putNumber("Indexer/Spindexer/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Indexer/Spindexer/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Indexer/Spindexer/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Indexer/Spindexer/Current", getCurrent().in(Amps));
    }
}
