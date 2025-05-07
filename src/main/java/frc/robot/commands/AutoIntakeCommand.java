package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHandlerSubsystem;

public class AutoIntakeCommand extends Command{
    private final CoralHandlerSubsystem coralHandlerSubsystem;
    public AutoIntakeCommand(CoralHandlerSubsystem coralHandlerSubsystem){
        this.coralHandlerSubsystem = coralHandlerSubsystem;
    }    

    @Override
    public void execute() {
        return;
    }

    @Override
    public boolean isFinished() {
        return this.coralHandlerSubsystem.isHopperBroken();
    }
}
