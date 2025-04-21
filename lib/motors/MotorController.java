package frc.robot.modules.motors;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A abstracted approach to motor controllers. Supports limit switches and following other motor controllers.
 */
public abstract class MotorController {

    Trigger positiveLSTrig;
    Trigger negativeLSTrig;

    private double motorUpperLimit = 1;
    private double motorLowerLimit = -1;

    private boolean following = false;
    private boolean followingInverted = false;

    private List<MotorController> followers;

    /**
     * Set the limit switches of this motor controller. Note that if this controller already has its limit switches set,
     * this method will do nothing.
     * @param positiveLS A lambda returning whether the positive limit switch is pressed. The positive limit switch is
     * the switch that is pressed when this motor controller is run in the positive direction.
     * @param negativeLS A lambda returning whether the negative limit switch is pressed. The negative limit switch is
     * the switch that is pressed when this motor controller is run in the negative direction.
     * @return This motor controller for method chaining.
     */
    public final MotorController withLimitSwitches(BooleanSupplier positiveLS, BooleanSupplier negativeLS) {
        Trigger posTrig = null;
        Trigger negTrig = null;

        if (positiveLS != null) {
            posTrig = new Trigger(positiveLS);
        }

        if (negativeLS != null) {
            negTrig = new Trigger(negativeLS);
        }

        withLimitSwitches(posTrig, negTrig);
        return this;
    }

    /**
     * Set the limit switches of this motor controller. Note that if this controller already has its limit switches set,
     * this method will do nothing.
     * @param positiveLS A {@link Trigger} which turns to true when the positive limit switch is pressed. The positive limit switch is
     * the switch that is hit when this motor controller is run in the positive direction.
     * @param negativeLS A {@link Trigger} which turns to true when the negative limit switch is pressed. The negative limit switch is
     * the switch that is hit when this motor controller is run in the negative direction.
     * @return This motor controller for method chaining.
     */
    public final MotorController withLimitSwitches(Trigger positiveLS, Trigger negativeLS) {

        // When an LS is pressed, change the allowed range of motor speeds
        if (positiveLS != null && positiveLSTrig == null) {
            this.positiveLSTrig = positiveLS;

            positiveLS.onTrue(new InstantCommand( () -> { motorUpperLimit = 0; setSpeed(getSpeed()); } ));
            positiveLS.onFalse(new InstantCommand( () -> { motorUpperLimit = 1; setSpeed(getSpeed()); } ));
        }

        if (negativeLS != null && positiveLSTrig == null) {
            this.negativeLSTrig = negativeLS;

            negativeLS.onTrue(new InstantCommand( () -> { motorLowerLimit = 0; setSpeed(getSpeed()); } ));
            negativeLS.onFalse(new InstantCommand( () -> { motorLowerLimit = -1; setSpeed(getSpeed()); } ));
        }
        return this;
    }
    
    /**
     * Gets the current speed (from 1 to -1) of the controller.
     */
    public abstract double getSpeed();

    /**
     * Sets the speed of the controller.
     * @param speed
     */
    protected abstract void setActualSpeed(double speed);

    /**
     * Sets the speed of the controller and updates its followers, while taking into account limit switches.
     * @param speed
     */
    public void setSpeed(double speed) {
        if (following) return;
        
        var actualSpeed = Math.max(motorLowerLimit, Math.min(motorUpperLimit, speed));
        setActualSpeed(actualSpeed);
        updateSelfAndFollowers(actualSpeed);
    }

    /**
     * Makes this motor controller follow another motor controller, with support for inversion.
     * @param leader The motor controller to follow.
     * @param inverted Whether to drive the opposite direction of the leader.
     */
    public final void follow(MotorController leader, boolean inverted) {
        if (following) {
            throw new IllegalStateException("This MotorController is already following another MotorController!");
        }
        following = true;
        followingInverted = inverted;

        leader.followers.add(this);

        onFollow(leader, inverted);
    }

    /**
     * Override this method in order to call code when this motor controller follows another controller.
     */
    protected void onFollow(MotorController leader, boolean inverted) {}

    /**
     * Override this method in order to customize the way this controller follows other {@link MotorController}s.
     * This method is called whenever the leading motor controller changes speed.
     * @param leaderSpeed The latest speed of the leader controller.
     */
    protected void updateSelf(double leaderSpeed) {
        this.setActualSpeed(leaderSpeed * ((isFollowingInverted()) ? -1 : 1));
    }

    protected final void updateFollowers(double speed) {
        followers.forEach((follower) -> follower.updateSelfAndFollowers(speed * ((isFollowingInverted()) ? -1 : 1)));
    }

    protected final void updateSelfAndFollowers(double speed) {
        updateSelf(speed);
        updateFollowers(speed);
    }

    protected final boolean isFollowing() {
        return following;
    }

    protected final boolean isFollowingInverted() {
        return followingInverted;
    }
}
