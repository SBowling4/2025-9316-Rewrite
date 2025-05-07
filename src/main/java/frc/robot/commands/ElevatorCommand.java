package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private double targetPosition;
    private double adjustment;  // For manual adjustment

    // Constructor for set positions
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;
        this.adjustment = 0.0;

        addRequirements(elevatorSubsystem);
    }

    // TODO make manual adjustment work
    // Currently instead of setting the target position to the manual adjust it instead sets it to -.5 or .5
    //I actually just made some changes but they prob still will not work - needs testing
    /**
     * Manual
     * @param elevatorSubsystem
     * @param isUp manual up true, manual down false
     */
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean isUp) {
        //sam was here
        this.elevatorSubsystem = elevatorSubsystem;

        this.adjustment = isUp ? ElevatorSubsystem.MANUAL_ADJUST_INCREMENT  : -ElevatorSubsystem.MANUAL_ADJUST_INCREMENT;
        this.targetPosition = elevatorSubsystem.getElevatorPosition() + adjustment;
        System.out.println("Setting target position to adjustment: " + targetPosition);
        


        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {  
         System.out.println(adjustment);
         System.out.println("Moving Elevator to: " + targetPosition);
       
    }

    @Override
    public void execute() {
        //System.out.println("Executing: Moving to " + targetPosition);
        elevatorSubsystem.setElevatorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        //Carter - try this instead of returning false if manual fix doesn't work 
        //return Math.abs(elevatorSubsystem.getElevatorPosition() -  < 0.1);
        return false;
         
        //return true; // Manual adjustments finish immediately after the adjustment 
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }
}
