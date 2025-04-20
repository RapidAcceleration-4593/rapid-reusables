package frc.robot.modules.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.motors.MotorController;

public class MotorSubsystem extends SubsystemBase {
    
    MotorController controller;

    public double getSpeed() {
        return controller.getSpeed();
    }
    
    public void setSpeed(double speed) {
        controller.setSpeed(speed);
    }

    public class Builder {
        private MotorSubsystem product;

        public Builder() {
            product = new MotorSubsystem();
        }

        public Builder withName(String name) {
            product.setName(name);
            return this;
        }

        public Builder withMotorController(MotorController controller) {
            product.controller = controller;
            return this;
        }
    }
}
