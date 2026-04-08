package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Feeder extends SubsystemBase{
    private final TalonFX motor = new TalonFX(kFeederID);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();
    
    private AngularVelocity targetVelocity = RPM.of(0);
    private Trigger upToSpeed = new Trigger(() -> upToSpeed()).debounce(kFeederDebounceTime);

    private final VelocityVoltage velocityrequest = new VelocityVoltage(0);

    public Feeder(){
        motor.getConfigurator().apply(kFeederConfig);
        
        positionStatus.setUpdateFrequency(100);
        velocityStatus.setUpdateFrequency(100);
        voltageStatus.setUpdateFrequency(100);
        statorStatus.setUpdateFrequency(50);

        ParentDevice.optimizeBusUtilizationForAll(motor);

        SmartDashboard.putData("3) Indexer/Feeder/Subsystem", this);
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

        motor.setControl(velocityrequest.withVelocity(targetVelocity));
        
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

    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
    }

    public Command setVelocityC(AngularVelocity velocity){
        return run(()-> setVelocity(velocity)).withName("Set Velocity: " + velocity);    
    }

    public Command reverseC(){
        return run(()->setVelocity(kFeederReverseVelocity)).withName("Reverse");
    }

    public Command feedC(){
        return run(()->feed()).withName("Feed");
    }

    public void feed() {
        setVelocity(kFeederVelocity);
    }

    private boolean upToSpeed() {
        return Math.abs(targetVelocity.in(RPM) - getVelocity().in(RPM)) < kFeederVelocityTolerance.in(RPM); 
    }

    public Trigger upToSpeedT() {
        return upToSpeed;
    }

    public void changeTunable(){
        
    }
        
    public void log(){
        // SmartDashboard.putNumber("3) Indexer/Feeder/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("3) Indexer/Feeder/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("3) Indexer/Feeder/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("3) Indexer/Feeder/Current", getCurrent().in(Amps));
    }
    
    // Simulation
    DCMotorSim model = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(1.0 / kFeederMotor.withReduction(kFeederGearRatio).KvRadPerSecPerVolt, 0.001),
        kFeederMotor
    );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        model.setInputVoltage(motorSimState.getMotorVoltage());
		model.update(0.02);
       
        motorSimState.setRawRotorPosition(model.getAngularPosition().times(kFeederGearRatio));
        motorSimState.setRotorVelocity(model.getAngularVelocity().times(kFeederGearRatio));
        motorSimState.setRotorAcceleration(model.getAngularAcceleration().times(kFeederGearRatio));
    }   
}
