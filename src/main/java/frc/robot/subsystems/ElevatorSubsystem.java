package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Configs;
import frc.robot.util.constants.Constants;
import frc.robot.util.constants.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;


public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevatorMotor;
    private SparkMax elevatorMotor2;
    private Encoder elevatorEncoder;
    private PIDController elevatorPID;

    private final CoralSubsystem coralSubsystem = CoralSubsystem.getInstance();
    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

    private double elevatorPower;
    private double targetPosition;

    private double ticks = 0;

    private static ElevatorSubsystem instance;
    
    public ElevatorSubsystem() {
        // Initialize Motors
        elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);

        elevatorMotor.configure(Configs.getElevatorConfigLeft(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        elevatorMotor2.configure(Configs.getElevatorConfigRight(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        elevatorEncoder = new Encoder(ElevatorConstants.ELEVATOR_ENCODER_CHANNEL_A, ElevatorConstants.ELEVATOR_ENCODER_CHANNEL_B);

        elevatorPID = new PIDController(0.225, 0.001, 0.01); 
        elevatorPID.setTolerance(0.025); 

        Shuffleboard.getTab("Elevator").addDouble("Elevator Power", () -> elevatorPower);
        Shuffleboard.getTab("Elevator").addDouble("Elevator Position", () -> getElevatorPosition());
        Shuffleboard.getTab("Elevator").addDouble("Target Position", () -> targetPosition);
    }

    // Setpoint method for PID control
    public Command setElevatorPosition(double targetPosition) {
        if (isWithinBounds(targetPosition)) { 
            double elevatorPower = elevatorPID.calculate(getElevatorPosition(), targetPosition);
            this.elevatorPower = elevatorPower;
            if (coralSubsystem.getIntakeBeam() && getElevatorPosition() < 2){
                stop();
            } else {
                if (elevatorPower < 0){
                    return Commands.run(() -> elevatorMotor.set(elevatorPower/50), this);
                }else{
                    return Commands.run(() -> elevatorMotor.set(elevatorPower), this);
                }
            }
        } else {
            setLEDColor(Constants.LEDConstants.RED, "red");
            stop();
        }

        return new InstantCommand();
    }

    public void stop() {
        elevatorMotor.stopMotor();
    }

    public double getElevatorPosition() {
        double currentRawValue = -elevatorEncoder.get();
        
        ticks += currentRawValue;

        return (ticks / ElevatorConstants.TICKS_PER_INCH);
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public double getElevatorPower(){
        return elevatorPower;
    }

    public boolean isWithinBounds(double position) {
        return position >= ElevatorConstants.MIN_HEIGHT && position <= ElevatorConstants.MAX_HEIGHT;
    }

    private void setLEDColor(int[] color, String colorName) {
        // Placeholder for LED control
        ledSubsystem.changeLEDColor(color, colorName);
        System.out.println("LED Color: " + colorName);
    }

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystem();
        }
        return instance;
    }
}
