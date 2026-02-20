package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Flywheel extends SubsystemBase {
    public TalonFX leftMotor = new TalonFX(kLeftMotorID);
    public TalonFX rightMotor = new TalonFX(kRightMotorID);

    private AngularVelocity targetVelocity = RPM.of(0);

    private final StatusSignal<Angle> positionStatus = leftMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = leftMotor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = leftMotor.getStatorCurrent();

    private final VelocityDutyCycle velocityrequest = new VelocityDutyCycle(0);
    

    public Flywheel() {
        leftMotor.getConfigurator().apply(kFlywheelConfig);
        rightMotor.getConfigurator().apply(kFlywheelConfig);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        SmartDashboard.putData("Shooter/Flywheel/Subsystem", this);
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
        leftMotor.setControl(velocityrequest.withVelocity(targetVelocity));
        log();
    }

    public Angle getAngle(){
        return positionStatus.getValue();
    }

    public AngularVelocity getAngularVelocity(){
        return velocityStatus.getValue();
    }

    public AngularVelocity getTargetVelocity(){
        return targetVelocity;
    }

    public Voltage getVoltage(){
        return voltageStatus.getValue();
    }

    public Current getCurrent(){
        return statorStatus.getValue();
    }

    public void setVoltage(double voltage){
        leftMotor.setVoltage(voltage);        
    }

    public void setVelocity(AngularVelocity velocity){
        targetVelocity = velocity;
    }

    public boolean upToSpeed(){
        return Math.abs(targetVelocity.in(RPM) - getAngularVelocity().in(RPM)) < RPMTolerance.get();
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    public Command setVelocityC(AngularVelocity velocity){
        return runOnce(()-> setVelocity(velocity)).until(upToSpeedT()).withName("Set velocity: " + velocity);
    }

    public Trigger upToSpeedT(){
        return new Trigger(()-> upToSpeed()).debounce(flywheelDebounceTime.get());
    }


     public void changeTunable(){
        flywheelIdleRPM.poll();
        flywheelDebounceTime.poll();
        RPMTolerance.poll();
        flywheelkP.poll();
        flywheelkI.poll();
        flywheelkD.poll();
        flywheelkS.poll();
        flywheelkV.poll();
        flywheelkA.poll();

        int hash = hashCode();

        if (flywheelkP.hasChanged(hash) || flywheelkI.hasChanged(hash) || flywheelkD.hasChanged(hash) || flywheelkS.hasChanged(hash) || flywheelkV.hasChanged(hash) ||flywheelkA.hasChanged(hash)) {
            kFlywheelConfig.Slot0.kP = flywheelkP.get();
            kFlywheelConfig.Slot0.kI = flywheelkI.get();
            kFlywheelConfig.Slot0.kD = flywheelkD.get(); 
            kFlywheelConfig.Slot0.kS = flywheelkS.get();
            kFlywheelConfig.Slot0.kV = flywheelkV.get();
            kFlywheelConfig.Slot0.kA = flywheelkA.get(); 
            leftMotor.getConfigurator().apply(kFlywheelConfig.Slot0);
        }
    }

    public double wrapAngle(){
        double angle = getAngle().in(Degrees);
        angle %= 360;
        if (angle <= 0) {
            angle += 360;
        }
        return angle;
    }

    public void log(){
        SmartDashboard.putNumber("Shooter/Flywheel/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter/Flywheel/Wrapped Angle Degrees", wrapAngle());
        SmartDashboard.putNumber("Shooter/Flywheel/RPM", getAngularVelocity().in(RPM));
        // SmartDashboard.putNumber("Shooter/Flywheel/Wheel Radians", getAngularVelocity().in(RadiansPerSecond));
        SmartDashboard.putNumber("Shooter/Flywheel/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter/Flywheel/Target RPM", targetVelocity.in(RPM));
        SmartDashboard.putNumber("Shooter/Flywheel/Current", getCurrent().in(Amps));
        SmartDashboard.putBoolean("Shooter/Flywheel/Up to speed", upToSpeedT().getAsBoolean());
        SmartDashboard.putNumber("Shooter/Flywheel/Velocity Tolerance", RPMTolerance.get());
    }

    // Simulation
    FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            kFlywheelMomentOfInertia.in(KilogramSquareMeters),
            kFlywheelGearRatio
        ),
        DCMotor.getKrakenX60(2),
        kFlywheelGearRatio
    );

    DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            kFlywheelMomentOfInertia.in(KilogramSquareMeters),
            kFlywheelGearRatio
        ),
        DCMotor.getKrakenX60(2)
    );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = leftMotor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means

        motorSimState.setSupplyVoltage(leftMotor.getSupplyVoltage().getValue());//TODO: Add friction? Also, idk that the voltage should be accessed like this
        motorSim.setInputVoltage(motorSimState.getMotorVoltage());

        motorSim.update(0.02);

        motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * kFlywheelGearRatio);
        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60  * kFlywheelGearRatio);
        //                                           shaft RPM --> rotations per second --> motor rotations per second
        double voltage = motorSim.getInputVoltage();
        flywheelSim.setInput(voltage);
		flywheelSim.update(0.02);
    }   
}
