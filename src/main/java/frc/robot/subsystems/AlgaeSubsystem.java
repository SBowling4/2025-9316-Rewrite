package frc.robot.subsystems;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.jni.DistanceSensorJNIWrapper;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralHandlerConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeSubsystem extends SubsystemBase {
   private SparkMax motor;
   public LEDSubsystem ledSubsystem;

    public AlgaeSubsystem() {
        motor = new SparkMax(AlgaeConstants.MOTOR_ID, MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

    public void stop(){
        motor.stopMotor();
    }
}
