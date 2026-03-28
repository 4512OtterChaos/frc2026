package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class OCTrigger {

    /**
     * Creates a new debounced trigger from this trigger - it will become active
     * when this trigger has
     * been active for longer than the specified period.
     *
     * @param trigger        The trigger to debounce.
     * @param tunableSeconds The debounce period.
     * @return The debounced trigger (rising edges debounced only)
     */
    public static Trigger debounce(Trigger trigger, DoubleSupplier tunableSeconds) {
        return debounce(trigger, tunableSeconds, Debouncer.DebounceType.kRising);
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active
     * when this trigger has
     * been active for longer than the specified period.
     *
     * @param trigger        The trigger to debounce.
     * @param tunableSeconds The debounce period.
     * @param type           The debounce type.
     * @return The debounced trigger.
     */
    public static Trigger debounce(Trigger trigger, DoubleSupplier tunableSeconds, Debouncer.DebounceType type) {
        return new Trigger(
                new BooleanSupplier() {
                    final Debouncer m_debouncer = new Debouncer(tunableSeconds.getAsDouble(), type);

                    @Override
                    public boolean getAsBoolean() {
                        m_debouncer.setDebounceTime(tunableSeconds.getAsDouble());
                        return m_debouncer.calculate(trigger.getAsBoolean());
                    }
                });
    }
}
