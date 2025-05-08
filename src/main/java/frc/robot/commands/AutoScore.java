package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoScore extends Command {
    private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private final CoralSubsystem coralSubsystem = CoralSubsystem.getInstance();
    private final CommandSwerveDrivetrain drivetrain = RobotContainer.getDrivetrain();

    private final AutoAlign alignCommand = new AutoAlign();

    private double elevatorTarget;

    public AutoScore(double elevatorTarget) {
        this.elevatorTarget = elevatorTarget;

        addRequirements(elevatorSubsystem, coralSubsystem, drivetrain);
    }

    @Override
    public void initialize() {
        DataLogManager.log("AutoScore: Starting command");
        alignCommand.schedule();
        elevatorSubsystem.setElevatorPosition(elevatorTarget);
    }

    @Override
    public void execute() {
        if (alignCommand.isFinished() && elevatorSubsystem.atSetpoint()) {
            coralSubsystem.outtake().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return alignCommand.isFinished() && elevatorSubsystem.atSetpoint() && !coralSubsystem.getOuttakeBeam();
    }

    @Override
    public void end(boolean interrupted) {
        alignCommand.cancel();
        coralSubsystem.stopCoralHandler();
    }
}
