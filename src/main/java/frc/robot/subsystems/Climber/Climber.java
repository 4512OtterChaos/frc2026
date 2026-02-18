    package frc.robot.subsystems.Climber;

    import com.ctre.phoenix6.BaseStatusSignal;
    import com.ctre.phoenix6.StatusSignal;
    import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

        public Angle targetAngle = Degrees.of(0);

        private final StatusSignal<Angle> positionStatus = motor.getPosition();
        private final StatusSignal<AngularVelocity> velocityStatus = motor.getVelocity();
        private final StatusSignal<Voltage> voltageStatus = motor.getMotorVoltage();
        private final StatusSignal<Current> statorStatus = motor.getStatorCurrent();

        private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(false);


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
            motor.setControl(mmRequest.withPosition(targetAngle));
            log();
        }
        
        public Angle getHeight(){
            return Degrees.of(positionStatus.getValueAsDouble());
        }
        
        public Angle getTargetRotations() {
            return targetAngle;
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
        
        public void setHeight(Angle angle){
            angle = Rotations.of(MathUtil.clamp(angle.in(Rotations), minAngleRot.get(), maxAngleRot.get())); //TODO: fix climber safety before testing
            targetAngle = angle;
        }

        public boolean atTargetHeight(){
            return targetAngle.in(Degrees) - getHeight().in(Degrees) <= angleToleranceRot.get();
        }

        public Command setVoltageC(double voltage){
            return run(()-> setVoltage(voltage)).withName("Set voltage: " + voltage);
        }

        public Command setHeightC(Angle angle){
            return run(()-> setHeight(angle)).withName("Set height: " + angle);
        }

        public Command setMinHeightC(){
            return setHeightC(Rotations.of(minAngleRot.get()));
        }

        public Command setMaxHeightC(){
            return setHeightC(Rotations.of(maxAngleRot.get()));
        }

        public Trigger atTargetHeightT(){
            return new Trigger(()-> atTargetHeight()).debounce(debounceTime.get());
        }

        public void changeTunable(){
            maxAngleRot.poll();
            minAngleRot.poll();
            angleToleranceRot.poll();
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
            SmartDashboard.putNumber("Climber/Height", getHeight().in(Rotations));
            SmartDashboard.putNumber("Climber/Target Height", getTargetRotations().in(Rotations));
            SmartDashboard.putNumber("Climber/RPS", getVelocity().in(RotationsPerSecond));
            SmartDashboard.putNumber("Climber/Voltage", getVoltage().in(Volts));
            SmartDashboard.putNumber("Climber/Current", getCurrent().in(Amps));
            SmartDashboard.putBoolean("Climber/At Height", atTargetHeightT().getAsBoolean());
            SmartDashboard.putNumber("Climber/Height Tolerance", angleToleranceRot.get());
        }

        // Simulation
        ElevatorSim climberSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                climberWeight,
                shaftRad.in(Meters), //TODO: replace
                kGearRatio
            ),
            DCMotor.getKrakenX60(1),
            angleToHeight(Rotations.of(minAngleRot.get())).in(Meters),
            angleToHeight(Rotations.of(maxAngleRot.get())).in(Meters),
            true,
            angleToHeight(Rotations.of(minAngleRot.get())).in(Meters)
        );

        @Override
        public void simulationPeriodic() {
            var motorSim = motor.getSimState();
            motorSim.Orientation = ChassisReference.CounterClockwise_Positive;
            motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            climberSim.setInput(motorSim.getMotorVoltage());
            climberSim.update(0.02);

            motorSim.setRawRotorPosition(heightToAngle(Meters.of(climberSim.getPositionMeters())));
            motorSim.setRotorVelocity(heightToAngle(Meters.of(climberSim.getVelocityMetersPerSecond())).per(Second));
        }   
    }
    /*
    * Possibilities:
    *    Stop
    *    Is manual (variable)
    *    Is stalled
    *    Tunable numbers: DONE
    */