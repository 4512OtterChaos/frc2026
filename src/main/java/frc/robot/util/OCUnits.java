package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.AngularMomentumUnit;
import edu.wpi.first.units.LinearMomentumUnit;
import edu.wpi.first.units.MomentOfInertiaUnit;

public class OCUnits {
    public static final LinearMomentumUnit PoundsInchesPerSecond = Pounds.mult(InchesPerSecond);
    public static final AngularMomentumUnit PoundsInchesSquaredPerSecond = PoundsInchesPerSecond.mult(Inches);
    public static final MomentOfInertiaUnit PoundSquareInches = PoundsInchesSquaredPerSecond.per(RadiansPerSecond); //Has about 6 decimals of accuracy in conversion at least compared to google conversion
}
