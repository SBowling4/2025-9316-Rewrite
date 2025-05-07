package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.util.Configs;
import frc.robot.util.constants.Constants;
import frc.robot.util.constants.Constants.CoralHandlerConstants;
import frc.robot.util.constants.Constants.LEDConstants;

public class CoralSubsystem extends SubsystemBase {
    private SparkMax coralHandlerMotor;

    private DigitalInput hopperBeamBreak;
    private DigitalInput intakeBeamBreak;
    private DigitalInput outtakeBeamBreak;

    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();    

    private boolean isCoralInProcess = false;

    private static CoralSubsystem instance;

    public CoralSubsystem() {
        coralHandlerMotor = new SparkMax(CoralHandlerConstants.CORAL_HANDLER_MOTOR_ID, MotorType.kBrushless);

        hopperBeamBreak = new DigitalInput(CoralHandlerConstants.HOPPER_BEAM_BREAK_ID);
        intakeBeamBreak = new DigitalInput(CoralHandlerConstants.INTAKE_BEAM_BREAK_ID);
        outtakeBeamBreak = new DigitalInput(CoralHandlerConstants.OUTTAKE_BEAM_BREAK_ID);

        coralHandlerMotor.configure(Configs.getCoralConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters); // Ensure motor starts off
    }

    public Command intake() {
        ledSubsystem.changeLEDColor(Constants.LEDConstants.YELLOW, "yellow");

        return Commands.run(() -> coralHandlerMotor.set(1), this).andThen(Commands.run(() -> stopCoralHandler(), this));
    }

    public Command nudgeForwards(){
        ledSubsystem.changeLEDColor(Constants.LEDConstants.RED, "red");

        return Commands.run(() -> coralHandlerMotor.set(0.19), this).andThen(Commands.run(() -> stopCoralHandler(), this));
    }

    public Command nudgeBack(){
        ledSubsystem.changeLEDColor(Constants.LEDConstants.RED, "red");

        return Commands.run(() -> coralHandlerMotor.set(-0.19), this).andThen(Commands.run(() -> stopCoralHandler(), this));
    }

    public Command outtake() {
        ledSubsystem.changeLEDColor(LEDConstants.RED, "red");

        return Commands.run(() -> coralHandlerMotor.set(.31), this).andThen(Commands.run(() -> stopCoralHandler(), this));
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

    public void stopCoralHandler() {
        coralHandlerMotor.set(0);
        ledSubsystem.changeLEDColor(Constants.LEDConstants.GREEN,"Green");
    }

    public static CoralSubsystem getInstance() {
        if (instance == null) {
            instance = new CoralSubsystem();
        }
        return instance;
    }

    @Override
    public void periodic() {
        boolean hopperBroken = getHopperBeam();
        boolean intakeBroken = getIntakeBeam();
        boolean outtakeBroken = getOuttakeBeam();

        if (hopperBroken || intakeBroken) {
            isCoralInProcess = true;
            ledSubsystem.changeLEDColor(LEDConstants.BLUE,"Blue");
            intake().schedule();
        }

        if (outtakeBroken) {
            isCoralInProcess = false;

            intake().cancel();
            coralHandlerMotor.stopMotor();
            
            ledSubsystem.changeLEDColor(LEDConstants.GREEN, "Green");
        }
    }
}
