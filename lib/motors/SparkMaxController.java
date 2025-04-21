package frc.robot.modules.motors;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SparkMaxController extends MotorController {

    SparkMax controller;
    boolean followingSpark = false;

    public SparkMaxController(int canId, MotorType type) {
        this(new SparkMax(canId, type));
    }

    public SparkMaxController(SparkMax controller) {
        followingSpark = controller.isFollower();
        this.controller = controller;
    }

    public REVLibError configure(SparkMaxConfig config, ResetMode resetMode, PersistMode persistMode) {
        return controller.configure(config, resetMode, persistMode);
    } 

    @Override
    public double getSpeed() {
        return controller.get();
    }

    @Override
    protected void setActualSpeed(double speed) {
        controller.set(speed);
    }

    @Override
    protected void onFollow(MotorController leader, boolean inverted) {
        // If the leader is another SparkMax, then use integrated following logic.
        if (leader instanceof SparkMaxController) {
            followingSpark = true;
            var spark = ((SparkMaxController)leader).controller;
            var config = new SparkMaxConfig();

            config.follow(spark, inverted);
            controller.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    @Override
    protected void updateSelf(double leaderSpeed) {
        if (followingSpark) return; // Following other sparks is handled by integrated logic.
        super.updateSelf(leaderSpeed);
    }
}
