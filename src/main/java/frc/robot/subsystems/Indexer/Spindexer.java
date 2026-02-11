package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Flywheel;

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
        BaseStatusSignal.refreshAll(
            positionStatus, 
            velocityStatus,
            voltageStatus,
            statorStatus
        );
        changeTunable();
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
        voltage = MathUtil.clamp(voltage, 0, 12); //TODO: Check if necessary
        motor.setVoltage(voltage);
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);    
    }

    public Command spindexC(){
        return setVoltageC(spindexerVoltage.get()).withName("Spindex");
    }

    public void changeTunable(){
        spindexerVoltage.poll();
        spindexSlowVoltage.poll();
        spindexerkP.poll();
        spindexerkI.poll();
        spindexerkD.poll();
        spindexerkS.poll();
        spindexerkV.poll();
        spindexerkA.poll();

        int hash = hashCode();

        if (spindexerkP.hasChanged(hash) || spindexerkI.hasChanged(hash) || spindexerkD.hasChanged(hash) || spindexerkS.hasChanged(hash) || spindexerkV.hasChanged(hash) || spindexerkA.hasChanged(hash)) {
            kSpindexerConfig.Slot0.kP = spindexerkP.get();
            kSpindexerConfig.Slot0.kI = spindexerkI.get();
            kSpindexerConfig.Slot0.kD = spindexerkD.get(); 
            kSpindexerConfig.Slot0.kS = spindexerkS.get();
            kSpindexerConfig.Slot0.kV = spindexerkV.get();
            kSpindexerConfig.Slot0.kA = spindexerkA.get();
            motor.getConfigurator().apply(kSpindexerConfig.Slot0);
        }
    }

    public void log(){
        SmartDashboard.putNumber("Indexer/Spindexer/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Indexer/Spindexer/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Indexer/Spindexer/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Indexer/Spindexer/Current", getCurrent().in(Amps));
    }

    // SImulation
    FlywheelSim spindexerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1),
            kMomentOfInertia.in(KilogramSquareMeters),
            kSpindexerGearRatio
        ),
        DCMotor.getKrakenX60(1),
        kSpindexerGearRatio
        );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means

        double voltage = motorSimState.getMotorVoltage();
        spindexerSim.setInputVoltage(voltage);
		spindexerSim.update(0.02);

        double wheelRadPerSec = spindexerSim.getAngularVelocityRadPerSec();
        double wheelRps = wheelRadPerSec / (2.0 * Math.PI);
       
        motorSimState.setRotorVelocity(wheelRps);
        motorSimState.addRotorPosition(wheelRps * 0.02);
    }   
}
