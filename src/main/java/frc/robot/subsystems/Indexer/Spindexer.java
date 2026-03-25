package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

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

public class Spindexer extends SubsystemBase {
    private TalonFX motor = new TalonFX(kSpindexerID);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    public Spindexer(){
        motor.getConfigurator().apply(kSpindexerConfig);
        
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        statorStatus.setUpdateFrequency(50);

        ParentDevice.optimizeBusUtilizationForAll(motor);
        
        SmartDashboard.putData("3) Indexer/Spindexer/Subsystem", this);
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
        motor.setVoltage(voltage);
    }

    public void spindex() {
        setVoltage(spindexerVoltage.get());
    }

    public Command setVoltageC(double voltage){
        return run(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);    
    }

    public Command reverseC(){
        return run(()->setVoltage(spindexerReverseVoltage.get())).withName("Reverse");
    }

    public Command spindexC(){
        return run(()->spindex()).withName("Spindex");
    }

    public void changeTunable(){
        spindexerVoltage.poll();
        // spindexSlowVoltage.poll();
    }

    public void log(){
        // SmartDashboard.putNumber("3) Indexer/Spindexer/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("3) Indexer/Spindexer/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("3) Indexer/Spindexer/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("3) Indexer/Spindexer/Current", getCurrent().in(Amps));
    }

    // Simulation
    DCMotorSim model = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
                1.0 / kSpindexerMotor.withReduction(kSpindexerGearRatio).KvRadPerSecPerVolt,
                0.001),
        kSpindexerMotor
    );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        model.setInputVoltage(motorSimState.getMotorVoltage());
		model.update(0.02);
       
        motorSimState.setRawRotorPosition(model.getAngularPosition().times(kSpindexerGearRatio));
        motorSimState.setRotorVelocity(model.getAngularVelocity().times(kSpindexerGearRatio));
        motorSimState.setRotorAcceleration(model.getAngularAcceleration().times(kSpindexerGearRatio));
    }   
}
