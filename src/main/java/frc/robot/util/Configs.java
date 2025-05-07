package frc.robot.util;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.constants.Constants;
import frc.robot.util.constants.Constants.ElevatorConstants;

public class Configs {
    public static SparkMaxConfig getAlgaeConfig() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.GLOBAL_CURRENT_LIMIT_DEFAULT);

        return config;
    }

    public static SparkMaxConfig getElevatorConfigRight() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(60);
        

        return config;
    }

    public static SparkMaxConfig getElevatorConfigLeft() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(60);
        config.follow(ElevatorConstants.ELEVATOR_MOTOR_1_ID, true); // Inverted follow

        return config;
    }

    public static SparkMaxConfig getCoralConfig() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.GLOBAL_CURRENT_LIMIT_DEFAULT);

        return config;
    }
}
