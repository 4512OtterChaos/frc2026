package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private TalonFX intakeMotor = new TalonFX(kIntakeMotorID);
    private TalonFX fourBarMotor = new TalonFX(kFourBarMotorID);

    private final StatusSignal<Angle> intakePositionStatus = intakeMotor.getPosition();
    private final StatusSignal<AngularVelocity> intakeVelocityStatus = intakeMotor.getVelocity();
    private final StatusSignal<Voltage> intakeVoltageStatus = intakeMotor.getMotorVoltage();
    private final StatusSignal<Current> intakeStatorStatus = intakeMotor.getStatorCurrent();

    private final StatusSignal<Angle> fourBarPositionStatus = fourBarMotor.getPosition();
    private final StatusSignal<AngularVelocity> fourBarVelocityStatus = fourBarMotor.getVelocity();
    private final StatusSignal<Voltage> fourBarVoltageStatus = fourBarMotor.getMotorVoltage();
    private final StatusSignal<Current> fourBarStatorStatus = fourBarMotor.getStatorCurrent();

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(intakePositionStatus, intakeVelocityStatus, intakeVoltageStatus, intakeStatorStatus,
                                    fourBarPositionStatus, fourBarVelocityStatus, fourBarVoltageStatus, fourBarStatorStatus);
        log();
    }

    public Intake(){
        intakeMotor.getConfigurator().apply(kIntakeConfig);
        fourBarMotor.getConfigurator().apply(kFourBarConfig);
        SmartDashboard.putData("Intake/Subsystem", this);
    }

    public Angle getIntakeAngle() {
        return intakePositionStatus.getValue();
    }

    public AngularVelocity getIntakeVelocity(){
        return intakeVelocityStatus.getValue();
    }

    public Voltage getIntakeVoltage(){
        return intakeVoltageStatus.getValue();
    }

    public Current getIntakeCurrent(){
        return intakeStatorStatus.getValue();
    }

     public Angle getFourBarAngle() {
        return fourBarPositionStatus.getValue();
    }

    public AngularVelocity getFourBarVelocity(){
        return fourBarVelocityStatus.getValue();
    }

    public Voltage getFourBarVoltage(){
        return fourBarVoltageStatus.getValue();
    }

    public Current getFourBarCurrent(){
        return fourBarStatorStatus.getValue();
    }

    public void setVoltage(double intakeVoltage, double fourBarVoltage){
        intakeMotor.setVoltage(intakeVoltage);
        fourBarMotor.setVoltage(fourBarVoltage);
    }

    public Command setVoltageC(double intakeVoltage, double fourBarVoltage){
        return runOnce(()-> setVoltage(intakeVoltage, fourBarVoltage)).withName("Set Voltage: " + intakeVoltage + ", " + fourBarVoltage);    
    }

    public Command setIntakeVoltageInC(){
        return setVoltageC(kIntakeVoltageIn, 0).withName("Intake Voltage In");
    }

    public Command setIntakeVoltageOutC(){
        return setVoltageC(kIntakeVoltageOut, 0).withName("Intake Voltage Out");
    }

    public Command setFourBarVoltageInC(){
        return setVoltageC(0, kFourBarVoltageIn).withName("Four Bar Voltage In");
    }

    public Command setFourBarVoltageOutC(){
        return setVoltageC(0, kFourBarVoltageOut).withName("Four Bar Voltage Out");
    }

    public void log(){
        SmartDashboard.putNumber("Intake/Intake Motor/Angle Degrees", getIntakeAngle().in(Degrees));
        SmartDashboard.putNumber("Intake/Intake Motor/RPM", getIntakeVelocity().in(RPM));
        SmartDashboard.putNumber("Intake/Intake Motor/Voltage", getIntakeVoltage().in(Volts));
        SmartDashboard.putNumber("Intake/Intake Motor/Current", getIntakeCurrent().in(Amps));

        SmartDashboard.putNumber("Intake/4 Bar Motor/Angle Degrees", getFourBarAngle().in(Degrees));
        SmartDashboard.putNumber("Intake/4 Bar Motor/RPM", getFourBarVelocity().in(RPM));
        SmartDashboard.putNumber("Intake/4 Bar Motor/Voltage", getFourBarVoltage().in(Volts));
        SmartDashboard.putNumber("Intake/4 Bar Motor/Current", getFourBarCurrent().in(Amps));
    }
}
