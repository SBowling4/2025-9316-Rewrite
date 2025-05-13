// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// gamepadManipulator = xbox, and joystick now = xboxDrive
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.HashMap;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix.platform.can.AutocacheState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoScore;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Telemetry;
import frc.robot.util.constants.Constants;
import frc.robot.util.constants.TunerConstants;
import frc.robot.util.constants.Constants.ElevatorConstants;

import org.elasticsearch.action.index.IndexRequest;
import org.elasticsearch.client.RequestOptions;
import org.elasticsearch.client.RestHighLevelClient;
import org.elasticsearch.client.RestClient;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused") // For now :)
public class RobotContainer {
    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main Tab");
    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    public boolean driveState = true;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.075).withRotationalDeadband(MaxAngularRate * 0.075) // Add a 7.5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric visDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController driverXbox = new CommandXboxController(0);  //Driver Controller
    private final CommandXboxController manipulatorXbox = new CommandXboxController(1);      //Manipulator Controller
 

    private static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
    private final CoralSubsystem coralSubsystem = CoralSubsystem.getInstance();
    private final AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.getInstance();
    private static final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance(); 

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer(){
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(    
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverXbox.getLeftY() * MaxSpeed/ 6) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed/ 6) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate / 2) // Drive counterclockwise with negative X (left)
            )
        );

        driverXbox.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverXbox.getLeftY() * MaxSpeed / 1.5)
                    .withVelocityY(driverXbox.getLeftX() * MaxSpeed / 1.5)
                    .withRotationalRate(driverXbox.getRightX() * MaxAngularRate)            
            )
        );

        driverXbox.x().whileTrue(new AutoAlign());

        driverXbox.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        manipulatorXbox.leftTrigger().whileTrue(algaeSubsystem.intake());
        manipulatorXbox.leftTrigger().whileTrue(algaeSubsystem.outtake());

        // Elevator Controls
        manipulatorXbox.a().onTrue(elevatorSubsystem.setElevatorPosition(ElevatorConstants.INTAKE_POSITION));    
        manipulatorXbox.b().onTrue(new AutoScore(ElevatorConstants.L2_POSITION));       
        manipulatorXbox.x().onTrue(new AutoScore(ElevatorConstants.L3_POSITION));        
        manipulatorXbox.rightBumper().onTrue(elevatorSubsystem.setElevatorPosition(ElevatorConstants.LOW_ALGAE_POSITION));        
        manipulatorXbox.leftBumper().onTrue(elevatorSubsystem.setElevatorPosition(ElevatorConstants.HIGH_ALGAE_POSITION));      
        
        manipulatorXbox.y().whileTrue(coralSubsystem.nudgeForwards());
        manipulatorXbox.start().whileTrue(coralSubsystem.nudgeBack());
        manipulatorXbox.rightTrigger().whileTrue(coralSubsystem.outtake());
        
        manipulatorXbox.back().onTrue(Commands.runOnce(() -> elevatorSubsystem.stop(), elevatorSubsystem));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        if (autoChooser.getSelected() != null){
            return autoChooser.getSelected();
        } else {
            return Commands.print("No autonomous command configured, if a path was chosen, this is an error.");
        }
    }

    public static CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public static VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    
}
