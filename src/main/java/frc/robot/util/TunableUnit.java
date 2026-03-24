package frc.robot.util;
// Copyright (c) 2025 FRC 6328

// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or
 * value not in dashboard.
 */
public class TunableUnit<M extends Measure<U>, U extends Unit> extends TunableUnitBase<M, U> {
    private static HashMap<Class<?>, BiConsumer<Double, ?>> doubletoUnitMap = new HashMap<>();
    static{
        doubletoUnitMap.put(Class<>)
    }

    /**
     * Create a new TunableNumber with the default value
     *
     * @param key          Key on dashboard
     * @param defaultValue Default value
     */
    private TunableUnit(String key, M defaultValue, U preferedUnit, BiConsumer<Double, M> doubleToUnit) {
        super(key, defaultValue, preferedUnit, doubleToUnit);
    }

    public static <M extends Measure<U>, U extends Unit> TunableUnit<M,U> of(String key, M defaultValue, U preferedUnit){
        doubletoUnitMap.get(defaultValue.unit().getClass().getClass());
        return new TunableUnit<>(key, defaultValue, preferedUnit, (num, val)-> val = preferedUnit.of(num));
    }
}
