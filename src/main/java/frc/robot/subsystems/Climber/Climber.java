    package frc.robot.subsystems.Climber;

    import com.ctre.phoenix6.BaseStatusSignal;
    import com.ctre.phoenix6.StatusSignal;
    import com.ctre.phoenix6.controls.PositionVoltage;
    import com.ctre.phoenix6.hardware.TalonFX;
    import com.ctre.phoenix6.sim.ChassisReference;

    import edu.wpi.first.math.MathUtil;
    import edu.wpi.first.math.system.plant.DCMotor;
    import edu.wpi.first.math.system.plant.LinearSystemId;
    import edu.wpi.first.units.measure.Angle;
    import edu.wpi.first.units.measure.AngularVelocity;
    import edu.wpi.first.units.measure.Current;
    import edu.wpi.first.units.measure.Voltage;
    import edu.wpi.first.wpilibj.RobotController;
    import edu.wpi.first.wpilibj.simulation.ElevatorSim;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import edu.wpi.first.wpilibj2.command.button.Trigger;

    import static edu.wpi.first.units.Units.*;
    import static frc.robot.subsystems.Climber.ClimberConstants.*;

        
    public class Climber extends SubsystemBase {
        public TalonFX motor = new TalonFX(kMotorID);

        public Angle targetHeight = Degrees.of(0);

        private final StatusSignal<Angle> positionStatus = motor.getPosition();
        private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
        private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
        private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

        private final PositionVoltage voltageRequest = new PositionVoltage(0);


        public Climber(){
            motor.getConfigurator().apply(kConfig);
            SmartDashboard.putData("Climber/Subsystem", this);
            resetHeight(Degrees.of(0));
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
            motor.setControl(voltageRequest.withPosition(targetHeight));
        }
        
        public Angle getHeight(){
            return Degrees.of(positionStatus.getValueAsDouble());
        }
        
        public Angle getTargetRotations() {
            return targetHeight;
        }

        public AngularVelocity getVelocity(){
            return RotationsPerSecond.of(velocityStatus.getValueAsDouble());// TODO: check if this works
        }

        public Voltage getVoltage(){
            return voltageStatus.getValue();
        }

        public Current getCurrent(){
            return statorStatus.getValue();
        }

        public void resetHeight(Angle height){
            motor.setPosition(height.in(Degrees));
        }

        public void setVoltage(double voltage){
            motor.setVoltage(voltage);        
        }
        
        public void setHeight(Angle degrees){
            degrees = Degrees.of(MathUtil.clamp(degrees.in(Degrees), minHeight.get(), maxHeight.get())); //TODO: fix climber safety before testing
            targetHeight = degrees;
        }

        public boolean atTargetHeight(){
            return targetHeight.in(Degrees) - getHeight().in(Degrees) <= heightTolerance.get();
        }

        public Command setVoltageC(double voltage){
            return run(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
        }

        public Command setHeightC(Angle angle){
            return run(()-> setHeight(angle)).withName("Set height: " + angle);
        }

        public Command setMinHeightC(){
            return setHeightC(Degrees.of(minHeight.get()));
        }

        public Command setMaxHeightC(){
            return setHeightC(Degrees.of(maxHeight.get()));
        }

        public Trigger atTargetHeightT(){
            return new Trigger(()-> atTargetHeight()).debounce(debounceTime.get());
        }

        public void changeTunable(){
            maxHeight.poll();
            minHeight.poll();
            heightTolerance.poll();
            debounceTime.poll();
            kP.poll();
            kI.poll();
            kD.poll();
            kS.poll();
            kV.poll(); 
            kA.poll();
            
            int hash = hashCode();
        
            if (kP.hasChanged(hash) || kI.hasChanged(hash) || kD.hasChanged(hash) || kS.hasChanged(hash) || kV.hasChanged(hash) || kA.hasChanged(hash)) {
                kConfig.Slot0.kP = kP.get();
                kConfig.Slot0.kI = kI.get();
                kConfig.Slot0.kD = kD.get(); 
                kConfig.Slot0.kS = kS.get();
                kConfig.Slot0.kV = kV.get();
                kConfig.Slot0.kA = kA.get(); 
                motor.getConfigurator().apply(kConfig.Slot0);
            }   
        }


        public void log(){  
            SmartDashboard.putNumber("Climber/Current Height", getHeight().in(Degrees));
            SmartDashboard.putNumber("Climber/Target Height", getTargetRotations().in(Degrees));
            SmartDashboard.putNumber("Climber/RPS", getVelocity().in(RotationsPerSecond));
            SmartDashboard.putNumber("Climber/Voltage", getVoltage().in(Volts));
            SmartDashboard.putNumber("Climber/Current", getCurrent().in(Amps));
            SmartDashboard.putBoolean("Climber/At Height", atTargetHeightT().getAsBoolean());
            SmartDashboard.putNumber("Climber/Height Tolerance", heightTolerance.get());
        }

        // Simulation
        ElevatorSim climberSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                climberWeight,
                wheelRad, //TODO: replace
                kGearRatio
            ),
            DCMotor.getKrakenX60(1),
            minHeight.get(),
            maxHeight.get(),
            true,
            minHeight.get()
        );

        @Override
        public void simulationPeriodic() {
            var motorSim = motor.getSimState();
            motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
            motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            climberSim.setInput(motorSim.getMotorVoltage());
            climberSim.update(0.02);

            motorSim.setRawRotorPosition(carriageDistToMotorAngle(Meters.of(climberSim.getPositionMeters())));
            motorSim.setRotorVelocity(carriageDistToMotorAngle(Meters.of(climberSim.getVelocityMetersPerSecond())).per(Second));
        }   
    }
    /*
    * Possibilities:
    *    Stop
    *    Is manual (variable)
    *    Is stalled
    *    Tunable numbers: DONE
    */