package frc.robot.util;

import static frc.robot.util.FieldUtil.*;
import static frc.robot.util.RobotConstants.driveMotor;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.Drivetrain.OCDrivetrain;
import frc.robot.subsystems.Drivetrain.TunerConstants;

public class RobotConstants {
    private static final OCDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final Mass kRobotWeight = Kilograms.of(45.359);
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(8.874);
    public static final Distance kWheelRad = Inches.of(2);
    public static final LinearVelocity kMaxDriveVelocity = MetersPerSecond.of(6);
    public static DCMotor driveMotor = DCMotor.getKrakenX60(1);
    public static final ModuleConfig kModuleConfig = new ModuleConfig(kWheelRad, kMaxDriveVelocity, 1, driveMotor, Amps.of(40), 2);
    
    // Module Offsets
    public static final Translation2d FL = new Translation2d(0.254, 0.306);
    public static final Translation2d FR = new Translation2d(0.254, -0.306);
    public static final Translation2d BL = new Translation2d(-0.254, 0.306);
    public static final Translation2d BR = new Translation2d(-0.254, -0.306);
    public static int kPigeonID = 51; //TODO: change later?
    
    // public enum Module {
    //     FL(1, 2, 6, 1, -172.353, kTrackLength/2, kTrackWidth/2), // Front Right
    //     FR(2, 1, 5, 4, -104.414, kTrackLength/2, -kTrackWidth/2), // Back Left
    //     BL(3, 7, 3, 3, 68.554, -kTrackLength/2, kTrackWidth/2), // Front left
    //     BR(4, 4, 8, 2, 66.357, -kTrackLength/2, -kTrackWidth/2); // Back Right

    //     public final int moduleNum;
    //     public final int driveMotorID;
    //     public final int steerMotorID;
    //     public final int cancoderID;
    //     public final double angleOffset;
    //     public final Translation2d centerOffset;

    //     private Module(int moduleNum, int driveMotorID, int steerMotorID, int cancoderID, double angleOffset, double xOffset, double yOffset){
    //         this.moduleNum = moduleNum;
    //         this.driveMotorID = driveMotorID;
    //         this.steerMotorID = steerMotorID;
    //         this.cancoderID = cancoderID;
    //         this.angleOffset = angleOffset;
    //         centerOffset = new Translation2d(xOffset, yOffset);
    //     }
    // }

    
}
