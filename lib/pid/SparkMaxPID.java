package frc.robot.modules.pid;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

public class SparkMaxPID extends SetpointController {

    private ControlType controlType;
    private SparkMax motor;
    private double setpoint;
    private double errorTolerance;

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        motor.getClosedLoopController().setReference(setpoint, controlType);
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(motor.getEncoder().getPosition() - setpoint) < errorTolerance;
    }

    @Override
    public double getCurrentPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void overrideOutput(double speed) {
        motor.getClosedLoopController().setReference(Math.min(Math.max(speed, -1), 1), ControlType.kDutyCycle);
    }
    
}
