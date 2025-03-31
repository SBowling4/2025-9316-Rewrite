
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CoralHandlerConstants;

public class CoralHandlerSubsystem extends SubsystemBase {


    // Beam Break Sensors
    private final DigitalInput hopperBeamBreak = new DigitalInput(CoralHandlerConstants.HOPPER_BEAM_BREAK_ID);
    private final DigitalInput intakeBeamBreak = new DigitalInput(CoralHandlerConstants.INTAKE_BEAM_BREAK_ID);
    private final DigitalInput outtakeBeamBreak = new DigitalInput(CoralHandlerConstants.OUTTAKE_BEAM_BREAK_ID);

    // Coral Handler Motor
    private final SparkMax coralHandlerMotor = new SparkMax(CoralHandlerConstants.CORAL_HANDLER_MOTOR_ID, MotorType.kBrushless);

    // // Elevator Movement Lock
    // private boolean elevatorLocked = false;

    // // Failsafe Timer
    // private final Timer failsafeTimer = new Timer();
    // private final Timer reverseTimer = new Timer(); // Timer for reverse action in case of failsafe
    // private static final double FAILSAFE_TIMEOUT = 3.0; // 3-second timeout
    // private static final double REVERSE_DURATION = 1.0; // Reverse for 1 second

    // Track if coral is passing through
    private boolean isCoralInProcess = false;
    public LEDSubsystem ledSubsystem;
    


    public CoralHandlerSubsystem(LEDSubsystem 
    ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        SparkMaxConfig config_ = new SparkMaxConfig();
        config_.idleMode(SparkBaseConfig.IdleMode.kBrake);
        coralHandlerMotor.set(0);
        coralHandlerMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters); // Ensure motor starts off
    }

    // //Start Intake//
    public void startIntake() {
        coralHandlerMotor.set(1);
        setLEDColor(Constants.LEDConstants.YELLOW, "yellow");
        //System.out.println("Coral Handler Intake Active");
    }

    public void nudge(){
        coralHandlerMotor.set(0.19);
        setLEDColor(Constants.LEDConstants.RED, "red");

    }
    public void back(){
        coralHandlerMotor.set(-0.19);
        setLEDColor(Constants.LEDConstants.RED, "red");

    }

    //Start Outtake//
    public void startOuttake() {
        coralHandlerMotor.set(0.31);
        setLEDColor(Constants.LEDConstants.RED, "red");
        //System.out.println("Coral Handler Outtake Active");
    }

    // public void activateManualOverride() {
    //     elevatorLocked = false; // Unlock the elevator manually
    //     System.out.println("Manual Override Activated: Elevator Unlocked");
    // }


    @Override
    public void periodic() {
        // Beam break logic (Beam break is triggered when FALSE)
        boolean hopperBroken = !hopperBeamBreak.get();
        boolean intakeBroken = !intakeBeamBreak.get();
        boolean outtakeBroken = !outtakeBeamBreak.get();

        if (hopperBroken) {
            isCoralInProcess = true;
            setLEDColor(Constants.LEDConstants.BLUE,"Blue"); // Idle
            //System.out.println("[Coral Handler] Intake Started - Hopper Beam Broken");
            coralHandlerMotor.set(.2);
        }

        if (!intakeBroken && !hopperBroken && isCoralInProcess) {
            isCoralInProcess = false;
            coralHandlerMotor.set(0);
        }

        // if (isCoralInProcess){
        //     coralHandlerMotor.set(.35);
        // } else if (!isCoralInProcess) {
        //     coralHandlerMotor.set(0);
        // }

        // if (isCoralInProcess && !intakeBroken && hopperBroken) {
        //     coralHandlerMotor.set(0);
        //     isCoralInProcess = false;
        //     System.out.println("[Coral Handler] Intake Stopped - Coral has passed through");
        // }

        // // Hopper Beam Break → Start Intake
        // if (hopperBroken && !isCoralInProcess && !isReversing) {
        //     coralHandlerMotor.set(0.5); // Start motor
        //     elevatorLocked = true;
        //     isCoralInProcess = true;
        //     failsafeTimer.reset();
        //     failsafeTimer.start();
        //     setLEDColor("BLUE"); // Intake Active
        //     System.out.println("Coral detected at hopper. Intake started, elevator locked.");
        // }

        // // Intake Beam Break Cleared → Stop Intake
        // if (isCoralInProcess && !intakeBroken && !isReversing) {
        //     coralHandlerMotor.set(0); // Stop motor
        //     elevatorLocked = false;
        //     isCoralInProcess = false;
        //     failsafeTimer.stop();
        //     setLEDColor("YELLOW"); // Coral ready for scoring
        //     System.out.println("Coral cleared intake. Elevator unlocked.");
        // }

        // // Outtake Beam Break → Coral Fully Ejected
        // if (outtakeBroken) {
        //     setLEDColor("RED"); // Ejecting
        //     System.out.println("Coral fully ejected.");
        // }

        // // Failsafe Timeout → Trigger Reverse
        // if (isCoralInProcess && failsafeTimer.get() > FAILSAFE_TIMEOUT && !isReversing) {
        //     isReversing = true;
        //     reverseTimer.reset();
        //     reverseTimer.start();
        //     coralHandlerMotor.set(-0.5); // Reverse motor
        //     setLEDColor("PURPLE");
        //     System.out.println("FAILSAFE: Reversing motor for " + REVERSE_DURATION + " second to clear jam.");
        // }

        // // Complete Reverse → Resume Intake
        // if (isReversing && reverseTimer.get() > REVERSE_DURATION) {
        //     reverseTimer.stop();
        //     coralHandlerMotor.set(0.5); // Resume normal intake
        //     failsafeTimer.reset(); // Reset the failsafe timer
        //     failsafeTimer.start();
        //     isReversing = false;
        //     System.out.println("Reverse complete. Resuming normal intake.");
        //     setLEDColor("BLUE"); // Resume intake
        // }
    }


    public boolean isHopperBroken(){
        //return false;
        return !hopperBeamBreak.get();
    }

    public boolean isOuttakeBroken(){
        //return false;
        return !outtakeBeamBreak.get();
    }

    public boolean isIntakeBroken(){
        //return false;
        return !intakeBeamBreak.get();
    }

    // public boolean isElevatorLocked() {
    //     return elevatorLocked;
    // }

    // public boolean isReversing() {
    //     return isReversing;
    // }

    public boolean isCoralInProcess() {
        return isCoralInProcess;
    }

    private void setLEDColor(int[] color, String colorName) {
        // Placeholder for LED control
        ledSubsystem.changeLEDColor(color, colorName);
        //System.out.println("LED Color: " + colorName);
    }

    public void stopCoralHandler() {
        coralHandlerMotor.set(0);
        setLEDColor(Constants.LEDConstants.GREEN,"Green"); // Idle
        //System.out.println("Coral Handler Stopped");
    }
}
