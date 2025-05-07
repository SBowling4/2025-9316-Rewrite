package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

@SuppressWarnings("unused")
public class AlgaeCommand extends Command {
    private AlgaeSubsystem algaeSubsystem;

    AlgaeMode mode;

    public enum AlgaeMode {
        EJECT,
        INTAKE,
        NONE
    }

    /**
     * eject was first
     * @param algaeSubsystem
     * @param mode
     */
    public AlgaeCommand(AlgaeSubsystem algaeSubsystem, AlgaeMode mode){
        this.algaeSubsystem = algaeSubsystem;
        this.mode = mode;

        addRequirements(algaeSubsystem);
    }
    
    
 
    @Override
    public void initialize() {
        // algaeSubsystem.distanceSensor.setAutomaticMode(true);
    }

    
        // Reset any other variables to factory defaults
   
    @Override
    public void execute() {
        //algaeSubsystem.setSpeed(1);
        /* if the distance is over 1 meter or not determined, than the motor stops
         * if the distance is less than 1m and greater than 8 cm than the motor runs normally
         * if the distance is less than 8 cm then the motor runs slowly.
         */
        // double distance = algaeSubsystem.getDistanceSensor();
        if (mode == AlgaeMode.INTAKE){
            algaeSubsystem.setSpeed(0.4);
           
        }else{
            if (mode == AlgaeMode.EJECT){
                algaeSubsystem.setSpeed(-1);
           
            }else {
          
                // if (distance > 3 && distance <= 30) {
                //     algaeSubsystem.setSpeed(0.4);
                //     //System.out.println("Algae trying to pull in");
                // } else if (distance <= 3 && distance > 0) {
                //     algaeSubsystem.setSpeed(0.05);
                //     //System.out.println("Algae holding");
                // } else if (distance <= 0 || distance > 30) {
                //     algaeSubsystem.stop();
                //    // System.out.println("Algae too far to see :()");
                // }

            }
        }
       
    }

    // @Override
    // public boolean isFinished() {
        // double distance = algaeSubsystem.getDistanceSensor();
        // if (distance <= 0 && distance > 0) {
        //     //TODO: tune first value may be 7cm 
        //     return true;
        // } else if(distance < 0){
        //     CommandScheduler.getInstance().schedule(Commands.print("Invalid range"));
        //     return true;
        // }

        // return false;
    // }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stop();
    }
}
