package frc.robot.modules.pid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.modules.LimitSwitch;

/**
 * An base class describing self sufficient setpoint controllers, with limit switch support.
 * Non-integrated implementations which must be externally updated should extend from {@link RioSetpointController}.
 */
public abstract class SetpointController {

    LimitSwitch positiveLS;
    LimitSwitch negativeLS;

    /**
     * Set the limit switches of this controller. Note that if this controller already has its limit switches set,
     * this method will do nothing.
     * @param positiveLS The positive limit switch is the switch that is hit when this controller is run in the positive direction.
     * @param negativeLS The negative limit switch is the switch that is hit when this controller is run in the negative direction.
     * @return This controller for method chaining.
     */
    public final SetpointController withLimitSwitches(LimitSwitch positiveLS, LimitSwitch negativeLS) {

        // When an LS is pressed, change the allowed range of PID speeds
        if (positiveLS != null && this.positiveLS == null) {
            this.positiveLS = positiveLS;

            positiveLS.asTrigger().whileTrue(new InstantCommand( () -> {
                setSetpoint(Math.min(getSetpoint(), getCurrentPosition()));
                if (positiveLS.doesResetEncoders()) {
                    resetPosition();
                }
            }));
        }

        if (negativeLS != null && this.negativeLS == null) {
            this.negativeLS = negativeLS;

            negativeLS.asTrigger().whileTrue(new InstantCommand( () -> {
                setSetpoint(Math.max(getSetpoint(), getCurrentPosition()));
                if (negativeLS.doesResetEncoders()) {
                    resetPosition();
                }
            }));
        }
        return this;
    }

    /**
     * Gets the setpoint this controller.
     * @return The setpoint of the controller. 
     */
    public abstract double getSetpoint();

    /**
     * Sets the desired position (setpoint) of this controller.
     * @param setpoint The desired position that this controller will attempt to move to.
     */
    public abstract void setSetpoint(double setpoint);

    /**
     * Is this controller at its setpoint?
     * @return True, if this controller is at its setpoint, false otherwise.
     */
    public abstract boolean atSetpoint();
     
    /**
     * The current position, which should be increased when the output of {@link #calculateOutput()} is positive,
     * and decreased when the output is negative.
     * @return
     */
    public abstract double getCurrentPosition();

    /**
     * Sets the current position to zero.
     */
    public abstract void resetPosition();

    /**
     * Overrides the PID output with this value, until {@link #setSetpoint()} is next called. 
     * @param speed The speed that is overriding the PID output. -1 to 1.
     */
    public abstract void overrideOutput(double speed);
}
