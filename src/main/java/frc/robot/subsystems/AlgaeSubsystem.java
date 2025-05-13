package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Configs;
import frc.robot.util.constants.Constants.AlgaeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeSubsystem extends SubsystemBase {
   private SparkMax motor;
   public LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

    private static AlgaeSubsystem instance;

    public AlgaeSubsystem() {
        motor = new SparkMax(AlgaeConstants.MOTOR_ID, MotorType.kBrushless);
        motor.configure(Configs.getAlgaeConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intake() {
        return Commands.startEnd(() -> motor.set(1), () -> stop(), this);
    }

    public Command outtake() {
        return Commands.startEnd(() -> motor.set(-1), () -> stop(), this);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

    public void stop(){
        motor.stopMotor();
    }

    public static AlgaeSubsystem getInstance() {
        if (instance == null) {
            instance = new AlgaeSubsystem();
        }
        return instance;
    }
}
