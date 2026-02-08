package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.subsystems.Intake.IntakeConstants.*;
import static frc.robot.util.OCUnits.PoundSquareInches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
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
        if (getAngle().in(Degrees) >= fourBarMaxDegrees.get()) {
            voltage = MathUtil.clamp(voltage, -12, 0);
        } 
        else if (getAngle().in(Degrees) <= fourBarMinDegrees.get()) {
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
            waitUntil(atAngle(Degrees.of(fourBarMaxDegrees.get()))),
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

    
    // Simulation
    FlywheelSim fourBarSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            kMomentOfInertia.in(KilogramSquareMeters),
            kFourBarGearRatio
        ),
        DCMotor.getKrakenX60(1)
    );


    DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            kMomentOfInertia.in(KilogramSquareMeters),
            kFourBarGearRatio
        ),
        DCMotor.getKrakenX60(2)
    );

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSimState = motor.getSimState();
        motorSimState.Orientation =  ChassisReference.Clockwise_Positive;//TODO: Fix, idk what it means

        motorSimState.setSupplyVoltage(motor.getSupplyVoltage().getValue());//TODO: Add friction? Also, idk that the voltage should be accessed like this
        motorSim.setInputVoltage(motorSimState.getMotorVoltage());

        motorSim.update(0.02);

        motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * kFourBarGearRatio);
        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60  * kFourBarGearRatio);
        //                                           shaft RPM --> rotations per second --> motor rotations per second
        double voltage = motorSim.getInputVoltage();
        fourBarSim.setInput(voltage);
		fourBarSim.update(0.02);
    }   
}