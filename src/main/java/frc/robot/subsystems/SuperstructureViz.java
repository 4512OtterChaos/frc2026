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
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Indexer.Feeder;
import frc.robot.subsystems.Indexer.Spindexer;

public class SuperstructureViz extends SubsystemBase{
    private OCDrivetrain drivetrain;
    private Intake intake;
    private FourBar fourBar;
    private Spindexer spindexer;
    private Feeder feeder;
    private Shooter shooter;

    private final Color8Bit kWindowColor = new Color8Bit(0, 100, 150);
    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);
    private final Color8Bit kSetpointBaseColor = new Color8Bit(149, 0, 67);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultHoodDeg = 90;
    private final double kDefaultFourBar1Deg = 90;
    private final double kDefaultFourBar2Deg = 90-3;

    public SuperstructureViz(OCDrivetrain drivetrain, Intake intake, FourBar fourBar, Spindexer spindexer, Feeder feeder, Shooter shooter){
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.fourBar = fourBar;
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.shooter = shooter;
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
    
    @Override
    public void simulationPeriodic() {
        hoodMech.setAngle(kDefaultHoodDeg - shooter.getHoodAngle().in(Degrees));
        hoodSetpoint.setAngle(kDefaultHoodDeg - shooter.getHoodTargetAngle().in(Degrees));

        fourBarMech1.setAngle(kDefaultFourBar1Deg - fourBar.getAngle().in(Degrees));

        fourBarMech2.setAngle(kDefaultFourBar2Deg - fourBar.getAngle().in(Degrees));
        
        SmartDashboard.putData("Hood Mech", hoodMechWindow);

        SmartDashboard.putData("FourBar Mech", fourBarMechWindow);
    }
}
