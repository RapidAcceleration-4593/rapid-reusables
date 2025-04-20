package frc.robot.modules.pid;

import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.modules.LimitSwitch;
import frc.robot.modules.motors.MotorController;

/**
 * A highly customizable class for {@link SetpointController} implementations with setpoint control loops executing on the roboRIO.
 * Instances must be constructed using a {@link Builder} object.
*/
public class RioSetpointController extends SetpointController {
    
    private Command executeCommand;

    private boolean overridden = true;

    private BiFunction<Double, Double, Double> calculator;
    private DoubleConsumer outputSetter;

    private DoubleSupplier setpointGetter;
    private DoubleConsumer setpointSetter;

    private BooleanSupplier atSetpointGetter;
    
    private DoubleSupplier currentPositionGetter;
    private Runnable resetPosition;

    private RioSetpointController() {}

    private void initialize() {
        // Because this controller is neither a subsystem nor a command, we must take things into our own hands.
        this.executeCommand = new FunctionalCommand(
            () -> {},
            this::update,
            this::onFinish,
            () -> false,
            (Subsystem)null);
        
        CommandScheduler.getInstance().schedule(executeCommand);

        overrideOutput(0);
    }

    // --- Lifecycle Methods ---

    private void update() {
        if (DriverStation.isDisabled()) {
            overrideOutput(0);
            return;
        }
        
        if (!overridden) {
            outputSetter.accept(calculator.apply(getCurrentPosition(), getSetpoint()));
        }
    }

    private void onFinish(boolean interrupted) {
        overrideOutput(0);
        CommandScheduler.getInstance().schedule(executeCommand); // On the bright side, update() will always be called.
    }

    // --- Overridden Methods ---

    @Override
    public void overrideOutput(double speed) {
        overridden = true;
        outputSetter.accept(speed);
    }

    @Override
    public double getSetpoint() {
        return setpointGetter.getAsDouble();
    }

    @Override
    public void setSetpoint(double setpoint) {
        overridden = false;
        setpointSetter.accept(setpoint);
    }

    @Override
    public boolean atSetpoint() {
        return atSetpointGetter.getAsBoolean();
    }

    @Override
    public double getCurrentPosition() {
        return currentPositionGetter.getAsDouble();
    }

    @Override
    public void resetPosition() {
        resetPosition.run();
    }

    // --- Builder Pattern ---
    
    // Unfortunately, for the Builder to be able to access the private members of the product, it has to be an inner class of the product.
    public static class Builder {
        
        private RioSetpointController product;
        private boolean finished = false;

        public Builder() {
            product = new RioSetpointController();
        }

        // --- General Construction Methods ---
        public Builder withSetpointHandling(DoubleSupplier setpointGetter, DoubleConsumer setpointSetter, BooleanSupplier atSetpointGetter) {
            checkFinished();
            product.setpointGetter = setpointGetter;
            product.setpointSetter = setpointSetter;
            product.atSetpointGetter = atSetpointGetter;
            return this;
        }

        public Builder withOutputCalculator(BiFunction<Double, Double, Double> calculator) {
            checkFinished();
            product.calculator = calculator;
            return this;
        }
        
        public Builder withOutput(DoubleConsumer output) {
            checkFinished();
            product.outputSetter = output;
            return this;
        }

        public Builder withCurrentPositionSupplier(DoubleSupplier currentPositionGetter, Runnable resetPosition) {
            checkFinished();
            product.currentPositionGetter = currentPositionGetter;
            product.resetPosition = resetPosition;
            return this;
        }
        
        // --- Concrete implementation methods ---

        public Builder usingEncoder(Encoder encoder) {
            checkFinished();
            withCurrentPositionSupplier(encoder::get, encoder::reset);
            return this;
        }

        public Builder usingMotorController(MotorController controller) {
            checkFinished();
            withOutput(controller::setSpeed);
            return this;
        }

        public Builder usingStandardPID(PIDConstants constants) {
            checkFinished();
            var pid = new PIDController(constants.kP, constants.kI, constants.kD);
            withOutputCalculator(pid::calculate);
            withSetpointHandling(pid::getSetpoint, pid::setSetpoint, pid::atSetpoint);
            return this;
        }

        public Builder usingProfiledPID(PIDConstants constants, Constraints constraints) {
            checkFinished();
            var pid = new ProfiledPIDController(constants.kP, constants.kI, constants.kD, constraints);
            withOutputCalculator(pid::calculate);
            withSetpointHandling(() -> pid.getGoal().position, pid::setGoal, pid::atSetpoint);
            return this;
        }

        public Builder withLimitSwitches(LimitSwitch positiveLS, LimitSwitch negativeLS) {
            checkFinished();
            product.withLimitSwitches(positiveLS, negativeLS);
            return this;
        }

        private void checkFinished() {
            if (finished) {
                throw new IllegalStateException("Product has already been retrieved!");
            }
        }

        public RioSetpointController retrieveProduct() {
            finished = true;
            product.initialize();
            return product;
        }
    }
}