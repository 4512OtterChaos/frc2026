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
public class TunableUnitBase<M extends Measure<U>, U extends Unit> implements DoubleSupplier {
    private static final String tableKey = "/Tuning";

    private final String key;
    private final DoubleEntry entry;
    private M defaultValue;
    private M valueUnit;
    private double valueDouble;
    private BiConsumer<Double, M> doubleToUnit;
    private U preferedUnit;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new TunableNumber with the default value
     *
     * @param key          Key on dashboard
     * @param defaultValue Default value
     */
    public TunableUnitBase(String key, M defaultValue, U preferedUnit, BiConsumer<Double, M> doubleToUnit) {
        this.key = tableKey + "/" + key;
        this.defaultValue = defaultValue;
        this.valueUnit = defaultValue;
        this.valueDouble = defaultValue.in(preferedUnit);
        this.preferedUnit = preferedUnit;
        this.doubleToUnit = doubleToUnit;
        this.entry = NetworkTableInstance.getDefault().getDoubleTopic(this.key).getEntry(0.0);
        entry.set(entry.get(defaultValue.in(preferedUnit)));
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public M get() {
        return valueUnit;
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double in(U unit) {
        return valueUnit.in(unit);
    }

    /**
     * Updates value. Must be called periodically.
     */
    public void poll() {
        valueDouble = entry.get(defaultValue.in(preferedUnit));
        doubleToUnit.accept(valueDouble, valueUnit);
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared
     *           between multiple
     *           objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was
     *         called, false
     *         otherwise.
     */
    public boolean hasChanged(int id) {
        double currentValue = in(preferedUnit);
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the tunableNumbers have changed
     *
     * @param id             Unique identifier for the caller to avoid conflicts
     *                       when shared between multiple *
     *                       objects. Recommended approach is to pass the result of
     *                       "hashCode()"
     * @param action         Callback to run when any of the tunable numbers have
     *                       changed. Access tunable
     *                       numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(
            int id, Consumer<double[]> action, TunableNumber... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableNumber::get).toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static void ifChanged(int id, Runnable action, TunableNumber... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble() {
        return in(preferedUnit);
    }
}
