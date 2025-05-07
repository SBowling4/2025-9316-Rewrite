package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHandlerSubsystem;

//Command to release Coral Stored at the beginning of the match at the current elevator level
public class AutoCoralStopCommand extends Command{
    private final CoralHandlerSubsystem coralHandlerSubsystem;
    public AutoCoralStopCommand(CoralHandlerSubsystem coralHandlerSubsystem){
        this.coralHandlerSubsystem = coralHandlerSubsystem;
    }

    @Override
    public void execute() {
        coralHandlerSubsystem.stopCoralHandler();
        System.out.println("Coral Handler Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
