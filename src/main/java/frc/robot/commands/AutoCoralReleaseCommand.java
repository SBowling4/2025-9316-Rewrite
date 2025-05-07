package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHandlerSubsystem;

//Command to release Coral Stored at the beginning of the match at the current elevator level
public class AutoCoralReleaseCommand extends Command{
    private final CoralHandlerSubsystem coralHandlerSubsystem;
    public AutoCoralReleaseCommand(CoralHandlerSubsystem coralHandlerSubsystem){
        this.coralHandlerSubsystem = coralHandlerSubsystem;
    }

    @Override
    public void execute() {
        coralHandlerSubsystem.startOuttake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
