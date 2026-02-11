package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Feeder extends SubsystemBase{
    private TalonFX motor = new TalonFX(kFeederID);

    private DigitalInput bottomSensor = new DigitalInput(0);
    private DigitalInput topSensor = new DigitalInput(1);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    public Feeder(){
        motor.getConfigurator().apply(kFeederConfig);
        SmartDashboard.putData("Indexer/Feeder/Subsystem", this);
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

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set Voltage: " + voltage);    
    }

    public Command feedC(){
        return setVoltageC(feederVoltage.get()).withName("Feed");
    }

    public Trigger bottomSensorT(){ // same as topSensorT
        return new Trigger(()-> bottomSensor.get());
    }

    public Trigger topSensorT(){ // might change to boolean to make indexC short :)
        return new Trigger(()-> topSensor.get());
    }

    public Command passiveIndexC(){
        return Commands.either(
            setVoltageC(feedSlowVoltage.get()),
            setVoltageC(0),
            ()-> !topSensorT().getAsBoolean() && bottomSensorT().getAsBoolean()
        ).withName("Passively Index");
    }

    public void changeTunable(){
        feederVoltage.poll();
        feedSlowVoltage.poll();
        feederkP.poll();
        feederkI.poll();
        feederkD.poll();
        feederkS.poll();
        feederkV.poll();
        feederkA.poll();
        int hash = hashCode();


        if (feederkP.hasChanged(hash) || feederkI.hasChanged(hash) || feederkD.hasChanged(hash) || feederkS.hasChanged(hash) || feederkV.hasChanged(hash) || feederkA.hasChanged(hash)) {
            kFeederConfig.Slot0.kP = feederkP.get();
            kFeederConfig.Slot0.kI = feederkI.get();
            kFeederConfig.Slot0.kD = feederkD.get(); 
            kFeederConfig.Slot0.kS = feederkS.get();
            kFeederConfig.Slot0.kV = feederkV.get();
            kFeederConfig.Slot0.kA = feederkA.get();
            motor.getConfigurator().apply(kFeederConfig.Slot0);
        }
    }
        
    public void log(){
        SmartDashboard.putNumber("Indexer/Feeder/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Indexer/Feeder/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Indexer/Feeder/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Indexer/Feeder/Current", getCurrent().in(Amps));
    }
    
    // SImulation
    FlywheelSim feederSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            kMomentOfInertia.in(KilogramSquareMeters),
            kFeederGearRatio
        ),
        DCMotor.getKrakenX60(2),
        kFeederGearRatio
        );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means

        double voltage = motorSimState.getMotorVoltage();
        feederSim.setInputVoltage(voltage);
		feederSim.update(0.02);

        double wheelRadPerSec = feederSim.getAngularVelocityRadPerSec();
        double wheelRps = wheelRadPerSec / (2.0 * Math.PI);
       
        motorSimState.setRotorVelocity(wheelRps);
        motorSimState.addRotorPosition(wheelRps * 0.02);
    }   
}
