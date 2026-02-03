// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.Volts;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import frc.robot.subsystems.Intake.FourBar;
// import frc.robot.subsystems.Intake.Intake;
// import frc.robot.subsystems.Shooter.Flywheel;
// import frc.robot.subsystems.Shooter.Hood;
// import frc.robot.subsystems.Shooter.ShooterConstants;
// import frc.robot.subsystems.Drivetrain.OCDrivetrain;
// import frc.robot.subsystems.Indexer.Feeder;
// import frc.robot.subsystems.Indexer.Spindexer;

// public class SuperstructureViz {
//     OCDrivetrain drivetrain;
//     Intake intake;
//     FourBar fourBar;
//     Spindexer spindexer;
//     Feeder feeder;
//     Flywheel flywheel;
//     Hood hood;

//     private final Color8Bit kSetpointBaseColor = new Color8Bit(150, 0, 0);
//     private final Color8Bit kMechBaseColor = new Color8Bit(0, 0, 150);

//     private final double kSetpointWidth = 6;
//     private final double kMechWidth = 9;

//     private final double kDefaultFunnelDeg = 90;
//     private final double kDefaultArmDeg = 88.186340;

//     public SuperstructureViz(OCDrivetrain drivetrain, Intake intake, FourBar fourBar, Spindexer spindexer, Feeder feeder, Flywheel flywheel, Hood hood){
//         this.drivetrain = drivetrain;
//         this.intake = intake;
//         this.fourBar = fourBar;
//         this.spindexer = spindexer;
//         this.feeder = feeder;
//         this.flywheel = flywheel;
//         this.hood = hood;
//     }

//     Mechanism2d hoodMech = new Mechanism2d(.4, .4, new Color8Bit(0, 100, 150));
//     MechanismRoot2d hoodMechRoot = hoodMech.getRoot("funnel", 0.3, 0.05);

//     private final MechanismLigament2d mechBase = hoodMech.append(
//             new MechanismLigament2d("Base", ShooterConstants.kPivotHeight.in(Meters), 90, kMechWidth, kMechBaseColor));
//     private final MechanismLigament2d mechFunnel = mechBase.append(
//             new MechanismLigament2d("Funnel", FunnelConstants.kHoodLength.in(Meters), kDefaultFunnelDeg, kMechWidth, kMechBaseColor));
    
//     private final MechanismLigament2d setpointBase = funnelMechRoot.append(
//             new MechanismLigament2d("setpointBase", ShooterConstants.kPivotHeight.in(Meters), 90, kSetpointWidth, kSetpointBaseColor));
//     private final MechanismLigament2d setpointFunnel = setpointBase.append(
//             new MechanismLigament2d("setpointFunnel", ShooterConstants.kArmLength.in(Meters), kDefaultFunnelDeg, kSetpointWidth,
//                     kSetpointBaseColor));
    


//     Mechanism2d shooterMech = new Mechanism2d(.5, .5, new Color8Bit(0, 100, 150));
//     MechanismRoot2d shooterMechRoot = shooterMech.getRoot("ShooterArm", 0.25, 0.05);

//     private final MechanismLigament2d mechBaseLeft = shooterMechRoot.append(
//             new MechanismLigament2d("ArmBaseLeft", ShooterArmConstants.kBaseLength.in(Meters)/2, 180, kMechWidth, kMechBaseColor));
//     private final MechanismLigament2d mechArmLeft = mechBaseLeft.append(
//             new MechanismLigament2d("ArmLeft", ShooterArmConstants.kPivotToWheels.in(Meters), -kDefaultArmDeg, kMechWidth, kMechBaseColor));
//     private final MechanismLigament2d mechBaseRight = shooterMechRoot.append(
//             new MechanismLigament2d("BaseRight", ShooterArmConstants.kBaseLength.in(Meters)/2, 0, kMechWidth, kMechBaseColor));
//     private final MechanismLigament2d mechArmRight = mechBaseRight.append(
//             new MechanismLigament2d("ArmRight", ShooterArmConstants.kPivotToWheels.in(Meters), kDefaultArmDeg, kMechWidth, kMechBaseColor));
    
//     private final MechanismLigament2d setpointBaseLeft = shooterMechRoot.append(
//             new MechanismLigament2d("setpointArmBaseLeft", ShooterArmConstants.kBaseLength.in(Meters)/2, 180, kSetpointWidth, kSetpointBaseColor));
//     private final MechanismLigament2d setpointArmLeft = setpointBaseLeft.append(
//             new MechanismLigament2d("setpointArmLeft", ShooterArmConstants.kPivotToWheels.in(Meters), -kDefaultArmDeg, kSetpointWidth,
//                     kSetpointBaseColor));
//     private final MechanismLigament2d setpointBaseRight = shooterMechRoot.append(
//             new MechanismLigament2d("setpointArmBaseRight", ShooterArmConstants.kBaseLength.in(Meters)/2, 0, kSetpointWidth, kSetpointBaseColor));
//     private final MechanismLigament2d setpointArmRight = setpointBaseRight.append(
//             new MechanismLigament2d("setpointArmRight", ShooterArmConstants.kPivotToWheels.in(Meters), kDefaultArmDeg, kSetpointWidth,
//                     kSetpointBaseColor));
    

    
    
//     public void periodic() {
//         mechFunnel.setAngle(90 - funnel.getAngle().in(Degrees));
//         setpointFunnel.setAngle(90 - funnel.getTargetAngle().in(Degrees));
        
//         mechArmLeft.setAngle(180 + (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));
//         mechArmRight.setAngle(180 - (kDefaultArmDeg - shooterArm.getAngle().in(Degrees)));
//         setpointArmLeft.setAngle(180 + (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
//         setpointArmRight.setAngle(180 - (kDefaultArmDeg - ((shooterArm.getTargetVoltage().in(Volts) > 0) ? ShooterArmConstants.kInAngle.in(Degrees) : ShooterArmConstants.kOutAngle.in(Degrees))));
        
//         SmartDashboard.putData("FunnelMech", funnelMech);
//         SmartDashboard.putData("ShooterMech", shooterMech);
//     }
// }
