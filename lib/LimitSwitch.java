package frc.robot.modules;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LimitSwitch {

    BooleanSupplier lsInput;
    Trigger trigger;

    boolean inverted;
    boolean resetsEncoders;

    /**
     * Creates a new limit switch object.
     * @param limitSwitch A lambda returning the limit switch's output.
     * @param inverted Does {@code limitSwitch} return false when the limit switch is pressed?
     * @param resetsEncoders Does this limit switch zero/reset encoders when it is pressed?
     */
    public LimitSwitch(BooleanSupplier limitSwitch, boolean inverted, boolean resetsEncoders) {
        this.lsInput = limitSwitch;
        this.inverted = inverted;
        this.resetsEncoders = resetsEncoders;
    }

    public boolean isPressed() {
        return lsInput.getAsBoolean() ^ inverted;
    }

    public boolean doesResetEncoders() {
        return resetsEncoders;
    }

    public Trigger asTrigger() {
        if (trigger == null) {
            trigger = new Trigger(lsInput);
        }
        return trigger;
    }
}
