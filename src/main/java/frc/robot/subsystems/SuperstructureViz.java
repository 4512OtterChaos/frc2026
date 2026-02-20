package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.FourBar;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;

public class SuperstructureViz extends SubsystemBase{
    private OCDrivetrain drivetrain;
    private Intake intake;
    private FourBar fourBar;
    private Spindexer spindexer;
    private Feeder feeder;
    private Flywheel flywheel;
    private Hood hood;
    private Climber climber;

    private final Color8Bit kWindowColor = new Color8Bit(0, 100, 150);
    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);
    private final Color8Bit kSetpointBaseColor = new Color8Bit(149, 0, 67);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultHoodDeg = 90;
    private final double kDefaultFourBar1Deg = 90; // TODO: elijah what do i do here
    private final double kDefaultFourBar2Deg = 90-3;

    public SuperstructureViz(OCDrivetrain drivetrain, Intake intake, FourBar fourBar, Spindexer spindexer, Feeder feeder, Flywheel flywheel, Hood hood, Climber climber){
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.fourBar = fourBar;
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = flywheel;
        this.hood = hood;
        this.climber = climber;
    }

    Mechanism2d hoodMechWindow = new Mechanism2d(0.4, 0.6, kWindowColor);
    MechanismRoot2d hoodMechRoot = hoodMechWindow.getRoot("Hood", 0.3, 0.05);

    private final MechanismLigament2d hoodMechBase = hoodMechRoot.append(
            new MechanismLigament2d("Hood Base", ShooterConstants.kHoodPivotHeight.in(Meters), 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d hoodMech = hoodMechBase.append(
            new MechanismLigament2d("Hood", ShooterConstants.kHoodLength.in(Meters), kDefaultHoodDeg, kMechWidth, kMechBaseColor));
    
    private final MechanismLigament2d hoodSetpointBase = hoodMechRoot.append(
            new MechanismLigament2d("Hood SetpointBase", ShooterConstants.kHoodPivotHeight.in(Meters), 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d hoodSetpoint = hoodSetpointBase.append(
            new MechanismLigament2d("Hood Setpoint", ShooterConstants.kHoodLength.in(Meters), kDefaultHoodDeg, kSetpointWidth, kSetpointBaseColor));


    Mechanism2d fourBarMechWindow = new Mechanism2d(0.8, 0.6, kWindowColor);
    MechanismRoot2d fourBarMech1Root = fourBarMechWindow.getRoot("Four Bar 1", 0.6, 0.05);
    MechanismRoot2d fourBarMech2Root = fourBarMechWindow.getRoot("Four Bar 2", 0.7, 0.05);

    private final MechanismLigament2d fourBarMech1Base = fourBarMech1Root.append(
            new MechanismLigament2d("FourBar 1 Base", IntakeConstants.kFourBar1PivotHeight.in(Meters), 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d fourBarMech1 = fourBarMech1Base.append(
            new MechanismLigament2d("FourBar 1", IntakeConstants.kFourBar1Length.in(Meters), kDefaultFourBar1Deg, kMechWidth, kMechBaseColor));

    private final MechanismLigament2d fourBarMech2Base = fourBarMech2Root.append(
            new MechanismLigament2d("FourBar 2 Base", IntakeConstants.kFourBar2PivotHeight.in(Meters), 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d fourBarMech2 = fourBarMech2Base.append(
            new MechanismLigament2d("FourBar 2", IntakeConstants.kFourBar2Length.in(Meters), kDefaultFourBar2Deg, kMechWidth, kMechBaseColor));
    
    private final MechanismLigament2d fourBarMech1SetpointBase = fourBarMech1Root.append(
            new MechanismLigament2d("FourBar 1 SetpointBase", IntakeConstants.kFourBar1PivotHeight.in(Meters), 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d fourBarMech1Setpoint = fourBarMech1SetpointBase.append(
            new MechanismLigament2d("FourBar 1 Setpoint", IntakeConstants.kFourBar1Length.in(Meters), kDefaultHoodDeg, kSetpointWidth, kSetpointBaseColor));

    private final MechanismLigament2d fourBarMech2SetpointBase = fourBarMech2Root.append(
            new MechanismLigament2d("FourBar 2 SetpointBase", IntakeConstants.kFourBar2PivotHeight.in(Meters), 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d fourBarMech2Setpoint = fourBarMech2SetpointBase.append(
            new MechanismLigament2d("FourBar 2 Setpoint", IntakeConstants.kFourBar2Length.in(Meters), kDefaultHoodDeg, kSetpointWidth, kSetpointBaseColor));

    
    @Override
    public void simulationPeriodic() {
        hoodMech.setAngle(kDefaultHoodDeg - hood.getAngle().in(Degrees));
        hoodSetpoint.setAngle(kDefaultHoodDeg - hood.getTargetAngle().in(Degrees));

        fourBarMech1.setAngle(kDefaultFourBar1Deg - fourBar.getAngle().in(Degrees));
        fourBarMech1Setpoint.setAngle(kDefaultFourBar1Deg - fourBar.getTargetAngle().in(Degrees));

        fourBarMech2.setAngle(kDefaultFourBar2Deg - fourBar.getAngle().in(Degrees));
        fourBarMech2Setpoint.setAngle(kDefaultFourBar2Deg - fourBar.getTargetAngle().in(Degrees));




        // mechArmLeft.setAngle(180 + (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));
        // mechArmRight.setAngle(180 - (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));
        // setpointArmLeft.setAngle(180 + (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
        // setpointArmRight.setAngle(180 - (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
        
        SmartDashboard.putData("Hood Mech", hoodMechWindow);

        SmartDashboard.putData("FourBar Mech", fourBarMechWindow);

        SmartDashboard.putData("Climber Mech", climberMechWindow);
        // SmartDashboard.putData("ShooterMech", shooterMech);
    }
}
