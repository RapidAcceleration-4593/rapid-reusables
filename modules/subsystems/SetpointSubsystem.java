package frc.robot.modules.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.ValueEnum;
import frc.robot.modules.pid.SetpointController;

/**
 * A generalized class that wraps a {@link SetpointController}.
 */
public class SetpointSubsystem extends SubsystemBase {

    protected SetpointController controller;

    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }

    public void setSetpoint(ValueEnum<Double> setpoint) {
        setSetpoint(setpoint.getConstantValue());
    }

    public double getSetpoint() {
        return controller.getSetpoint();
    }

    public void zeroEncoder() {
        controller.resetPosition();
    }

    public void manualDrive(double dutyCycle) {
        controller.overrideOutput(dutyCycle);
    }

    public class Builder {
        private SetpointSubsystem product;

        public Builder() {
            product = new SetpointSubsystem();
        }

        public Builder withName(String name) {
            product.setName(name);
            return this;
        }

        public Builder withSetpointController(SetpointController controller) {
            product.controller = controller;
            return this;
        }
    }
}
