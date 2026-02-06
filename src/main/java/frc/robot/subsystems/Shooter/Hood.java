package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;


public class Hood extends SubsystemBase {
    public TalonFX motor = new TalonFX(kHoodMotorID);

    private Angle targetAngle = Degrees.of(0);
 
    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);   
    
    public Hood(){
        motor.getConfigurator().apply(kHoodConfig);
        SmartDashboard.putData("Shooter/Hood/Subsystem", this);
        resetAngle(Degrees.of(hoodMinAngle.get()));
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(
            positionStatus,
            velocityStatus,
            voltageStatus,
            statorStatus
        );
        motor.setControl(mmRequest.withPosition(targetAngle));
        log();
    }
    
    public Angle getAngle(){
        return positionStatus.getValue();
    }

    public Angle getTargetAngle(){
        return targetAngle;
    }

    public AngularVelocity getAngularVelocity(){
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
        motor.setVoltage(voltage);        
    }

    public void setAngle(Angle angle){
        angle = Degrees.of(MathUtil.clamp(angle.in(Degrees), hoodMinAngle.get(), hoodMaxAngle.get()));
        targetAngle = angle;
    }
    
    public boolean atAngle(){
        return Math.abs(targetAngle.in(Degrees) - getAngle().in(Degrees)) < degreesTolerance.get();
    }

    public Command setVoltageC(double voltage){
        return runOnce(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
    }

    public Command setAngleC(Angle angle){
        return run(()-> setAngle(angle)).until(atAngleT()).withName("Set angle: " + angle);
    }

    public Command setMinAngleC(){
        return setAngleC(Degrees.of(hoodMinAngle.get()));
    }

    public Trigger atAngleT(){
        return new Trigger(()-> atAngle()).debounce(hoodDebounceTime.get());
    }

    public void changeTunable() {
        hoodMinAngle.poll();
        hoodMaxAngle.poll();
        hoodDebounceTime.poll();
        degreesTolerance.poll();
        hoodkP.poll();
        hoodkI.poll();
        hoodkD.poll();
        hoodkS.poll();
        hoodkV.poll();
        hoodkA.poll();

        int hash = hashCode();

        if (hoodkP.hasChanged(hash) || hoodkI.hasChanged(hash) || hoodkD.hasChanged(hash) || hoodkS.hasChanged(hash) || hoodkV.hasChanged(hash) || hoodkA.hasChanged(hash)) {
            kHoodConfig.Slot0.kP = hoodkP.get();
            kHoodConfig.Slot0.kI = hoodkI.get();
            kHoodConfig.Slot0.kD = hoodkD.get();
            kHoodConfig.Slot0.kS = hoodkS.get();
            kHoodConfig.Slot0.kV = hoodkV.get();
            kHoodConfig.Slot0.kA = hoodkA.get();
            motor.getConfigurator().apply(kHoodConfig.Slot0);
        }

    }

    public void log(){
        SmartDashboard.putNumber("Shooter/Hood/Angle", getAngle().in(Degrees));
        SmartDashboard.putNumber("Shooter/Hood/Target Angle", targetAngle.in(Degrees));
        SmartDashboard.putNumber("Shooter/Hood/RPM", getAngularVelocity().in(RPM));
        // SmartDashboard.putNumber("Shooter/Hood/Wheel Radians", getAngularVelocity().in(RadiansPerSecond));
        SmartDashboard.putNumber("Shooter/Hood/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Shooter/Hood/Current", getCurrent().in(Amps));
        SmartDashboard.putBoolean("Shooter/Hood/At Angle", atAngleT().getAsBoolean());
        SmartDashboard.putNumber("Shooter/Hood/Angle Tolerance", degreesTolerance.get());
    }


    // Simulation
    SingleJointedArmSim hoodSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            kHoodGearRatio,
            kMomentOfInertia.in(KilogramSquareMeters),
            kArmLength.in(Meters),
            kHoodMinAngle.in(Radians),
            kHoodMaxAngle.in(Radians),
            true,//TODO:Include gravity?
            kHoodMinAngle.in(Radians)
        );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation =  ChassisReference.CounterClockwise_Positive;//TODO: Fix, idk what it means

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double voltage = motorSimState.getMotorVoltage();
        hoodSim.setInputVoltage(voltage);
        hoodSim.update(0.02);

        double armRad = hoodSim.getAngleRads();
        double armRadPerSec = hoodSim.getVelocityRadPerSec();
        double armRot = armRad / (2.0 * Math.PI);
        double armRps = armRadPerSec / (2.0 * Math.PI);

        motorSimState.setRawRotorPosition(armRot);
        motorSimState.setRotorVelocity(armRps);
    }
}