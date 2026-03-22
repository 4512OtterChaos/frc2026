package frc.robot.util;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.Drivetrain.TunerConstants;

public class RobotConstants {
    public static final Mass kRobotWeight = Kilograms.of(100);
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(8.874);
    public static final Distance kWheelRad = Meters.of(TunerConstants.BackLeft.WheelRadius);
    public static final LinearVelocity kMaxDriveVelocity = TunerConstants.kSpeedAt12Volts;
    public static DCMotor driveMotor = DCMotor.getKrakenX60(1).withReduction(TunerConstants.BackLeft.DriveMotorGearRatio);
    public static final ModuleConfig kModuleConfig = new ModuleConfig(kWheelRad, kMaxDriveVelocity, 1, driveMotor, Amps.of(60), 1);
    
    // Module Offsets
    public static final Translation2d FL = new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);
    public static final Translation2d FR = new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY);
    public static final Translation2d BL = new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY);
    public static final Translation2d BR = new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY);

    public static final Translation2d kShooterTranslation = new Translation2d(Inches.of(-6.486584), Inches.of(-5.500000));

}
