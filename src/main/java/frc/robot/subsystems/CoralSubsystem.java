package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CoralHandlerConstants;
import frc.robot.Constants.LEDConstants;

public class CoralSubsystem extends SubsystemBase {
    private SparkMax coralHandlerMotor;

    private DigitalInput hopperBeamBreak;
    private DigitalInput intakeBeamBreak;
    private DigitalInput outtakeBeamBreak;

    private LEDSubsystem ledSubsystem;    

    private boolean isCoralInProcess = false;



    public CoralSubsystem() {
        coralHandlerMotor = new SparkMax(CoralHandlerConstants.CORAL_HANDLER_MOTOR_ID, MotorType.kBrushless);

        hopperBeamBreak = new DigitalInput(CoralHandlerConstants.HOPPER_BEAM_BREAK_ID);
        intakeBeamBreak = new DigitalInput(CoralHandlerConstants.INTAKE_BEAM_BREAK_ID);
        outtakeBeamBreak = new DigitalInput(CoralHandlerConstants.OUTTAKE_BEAM_BREAK_ID);

        SparkMaxConfig config_ = new SparkMaxConfig();
        config_.idleMode(SparkBaseConfig.IdleMode.kBrake);
        coralHandlerMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters); // Ensure motor starts off
    }

    public Command intake() {
        setLEDColor(Constants.LEDConstants.YELLOW, "yellow");

        return Commands.run(() -> coralHandlerMotor.set(1), this);
    }

    public Command nudgeForwards(){
        setLEDColor(Constants.LEDConstants.RED, "red");

        return Commands.run(() -> coralHandlerMotor.set(0.19), this);
    }
    public Command nudgeBack(){
        setLEDColor(Constants.LEDConstants.RED, "red");

        return Commands.run(() -> coralHandlerMotor.set(-0.19), this);
    }

    //Start Outtake//
    public void startOuttake() {
        coralHandlerMotor.set(0.31);
        setLEDColor(Constants.LEDConstants.RED, "red");
    }

    public boolean getHopperBeam(){
        return !hopperBeamBreak.get();
    }

    public boolean getOuttakeBeam(){
        return !outtakeBeamBreak.get();
    }

    public boolean getIntakeBeam(){
        return !intakeBeamBreak.get();
    }

    public boolean isCoralInProcess() {
        return isCoralInProcess;
    }

    private void setLEDColor(int[] color, String colorName) {
        // Placeholder for LED control
        ledSubsystem.changeLEDColor(color, colorName);
    }

    public void stopCoralHandler() {
        coralHandlerMotor.set(0);
        setLEDColor(Constants.LEDConstants.GREEN,"Green");
    }

    @Override
    public void periodic() {
        boolean hopperBroken = getHopperBeam();
        boolean intakeBroken = getIntakeBeam();
        boolean outtakeBroken = getOuttakeBeam();

        if (hopperBroken || intakeBroken) {
            isCoralInProcess = true;
            setLEDColor(LEDConstants.BLUE,"Blue");
            intake().schedule();
        }

        if (outtakeBroken) {
            isCoralInProcess = false;

            intake().cancel();
            coralHandlerMotor.stopMotor();
            
            setLEDColor(LEDConstants.GREEN, "Green");
        }
    }
}
