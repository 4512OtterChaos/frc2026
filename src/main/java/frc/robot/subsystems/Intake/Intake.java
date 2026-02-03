package frc.robot.subsystems.Intake;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Intake.IntakeConstants.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.kMomentOfInertia;

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

public class Intake extends SubsystemBase{
    private TalonFX motor = new TalonFX(kIntakeMotorID);

    private final StatusSignal<Angle> positionStatus = motor.getPosition();
    private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
    private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
    private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

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

    public Intake(){
        motor.getConfigurator().apply(kIntakeConfig);
        SmartDashboard.putData("Intake/Roller/Subsystem", this);
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

    public Command setVoltageInC(){
        return setVoltageC(intakeVoltageIn.get()).withName("Voltage In");
    }

    public Command setVoltageOutC(){
        return setVoltageC(intakeVoltageOut.get()).withName("Voltage Out");
    }

    public void changeTunable(){
        intakeVoltageIn.poll();
        intakeVoltageOut.poll();
    }

    public void log(){
        SmartDashboard.putNumber("Intake/Roller/Angle Degrees", getAngle().in(Degrees));
        SmartDashboard.putNumber("Intake/Roller/RPM", getVelocity().in(RPM));
        SmartDashboard.putNumber("Intake/Roller/Voltage", getVoltage().in(Volts));
        SmartDashboard.putNumber("Intake/Roller/Current", getCurrent().in(Amps));
    }

    // Simulation
    FlywheelSim intakeSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            kMomentOfInertia.in(KilogramSquareMeters),
            kIntakeGearRatio
        ),
        DCMotor.getKrakenX60(1)
        );


    DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            kMomentOfInertia.in(KilogramSquareMeters),
            kIntakeGearRatio
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

        motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * kIntakeGearRatio);
        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60  * kIntakeGearRatio);
        //                                           shaft RPM --> rotations per second --> motor rotations per second
        double voltage = motorSim.getInputVoltage();
        intakeSim.setInput(voltage);
		intakeSim.update(0.02);
    }   
}
