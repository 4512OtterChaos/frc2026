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

    private final Color8Bit kSetpointBaseColor = new Color8Bit(150, 0, 0);
    private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);

    private final double kSetpointWidth = 6;
    private final double kMechWidth = 9;

    private final double kDefaultHoodDeg = 90;
    // private final double kDefaultArmDeg = 88.186340;

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

    Mechanism2d hoodMech = new Mechanism2d(.4, .6, new Color8Bit(0, 100, 150));
    MechanismRoot2d hoodMechRoot = hoodMech.getRoot("Hood", 0.3, 0.05);

    private final MechanismLigament2d mechBase = hoodMechRoot.append(
            new MechanismLigament2d("Base", ShooterConstants.kHoodPivotHeight.in(Meters), 90, kMechWidth, kMechBaseColor));
    private final MechanismLigament2d mechHood = mechBase.append(
            new MechanismLigament2d("Hood", ShooterConstants.kHoodLength.in(Meters), kDefaultHoodDeg, kMechWidth, kMechBaseColor));
    
    private final MechanismLigament2d setpointBase = hoodMechRoot.append(
            new MechanismLigament2d("setpointBase", ShooterConstants.kHoodPivotHeight.in(Meters), 90, kSetpointWidth, kSetpointBaseColor));
    private final MechanismLigament2d setpointHood = setpointBase.append(
            new MechanismLigament2d("setpointHood", ShooterConstants.kHoodLength.in(Meters), kDefaultHoodDeg, kSetpointWidth, kSetpointBaseColor));


    Mechanism2d fourBarMech = new Mechanism2d(.8, .6, new Color8Bit(0, 100, 150));
    MechanismRoot2d fourBMechRoot = fourBarMech.getRoot("Four Bar", 0.7, 0.05);

    private final MechanismLigament2d 
    
    @Override
    public void simulationPeriodic() {
        mechHood.setAngle(kDefaultHoodDeg - hood.getAngle().in(Degrees));
        setpointHood.setAngle(kDefaultHoodDeg - hood.getTargetAngle().in(Degrees));
        
        // mechArmLeft.setAngle(180 + (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));
        // mechArmRight.setAngle(180 - (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));
        // setpointArmLeft.setAngle(180 + (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
        // setpointArmRight.setAngle(180 - (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
        
        SmartDashboard.putData("HoodMech", hoodMech);
        // SmartDashboard.putData("ShooterMech", shooterMech);
    }
}
