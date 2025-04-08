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
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AutoCoralReleaseCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.AlgaeCommand.AlgaeMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AutoSubsystem; //this thingy
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;


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

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController xboxDrive = new CommandXboxController(0);  //Driver Controller
    private final CommandXboxController gamepadManipulator = new CommandXboxController(1);      //Manipulator Controller
 

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

   
    
    public final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final CoralHandlerSubsystem coralHandler = new CoralHandlerSubsystem(ledSubsystem);
    public final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

    public final AutoSubsystem autoSubsystem = new AutoSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    
    
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(coralHandler, ledSubsystem); // Initialize Elevator Subsystem
    //private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer(){
        NamedCommands.registerCommand("AutoExchange", autoSubsystem.AutoExchange(coralHandler, elevatorSubsystem, algaeSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        NamedCommands.registerCommand("AutoVisionCommand", autoSubsystem.AutoVision(coralHandler, visionSubsystem, drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        NamedCommands.registerCommand("StopVision", autoSubsystem.StopVision(coralHandler, visionSubsystem, drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        //individual commands
        NamedCommands.registerCommand("AutoIntake", autoSubsystem.AutoIntake(coralHandler).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        NamedCommands.registerCommand("Scoral", autoSubsystem.Scoral(coralHandler));
        NamedCommands.registerCommand("L2Pos", autoSubsystem.L2Pos(elevatorSubsystem));
        NamedCommands.registerCommand("L1Pos", autoSubsystem.L1Pos(elevatorSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        NamedCommands.registerCommand("L1PosLONG", autoSubsystem.L1Pos(elevatorSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        NamedCommands.registerCommand("IntakePos", autoSubsystem.IntakePos(elevatorSubsystem));
        NamedCommands.registerCommand("Algae1Pos", autoSubsystem.IntakePos(elevatorSubsystem));

        NamedCommands.registerCommand("Algae2Pos", autoSubsystem.IntakePos(elevatorSubsystem));
        NamedCommands.registerCommand("GrabAlgae", autoSubsystem.GrabAlgae(algaeSubsystem));
        NamedCommands.registerCommand("EjectAlgae", autoSubsystem.EjectAlgae(algaeSubsystem));


        algaeSubsystem.setDefaultCommand(new AlgaeCommand(algaeSubsystem, AlgaeMode.NONE));
   
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureAutoChooser();
        setupShuffleboard();
         
       
    }

    private void setupShuffleboard() {
        //Beam Break Data
        mainTab.addBoolean("Hopper Beam Break", () -> coralHandler.isHopperBroken())
            .withPosition(0,0).withSize(2,1);

        mainTab.addBoolean("Intake Beam Break", () -> coralHandler.isIntakeBroken())
            .withPosition(0,1).withSize(2,1);

        mainTab.addBoolean("Outtake Beam Break", () -> coralHandler.isOuttakeBroken())
            .withPosition(0,2).withSize(2,1);

        // Elevator Data
        mainTab.addNumber("Elevator Position", () -> elevatorSubsystem.getElevatorPosition())
            .withPosition(2,0).withSize(2,1);

        mainTab.addNumber("Elevator Target", () -> elevatorSubsystem.getTargetPosition())
            .withPosition(4,0).withSize(2,1);

        mainTab.addNumber("Elevator Power", () -> elevatorSubsystem.getElevatorPower())
            .withPosition(4,2).withSize(2,1);
        //Coral Process
        mainTab.addBoolean("Is Coral In Process", () -> coralHandler.isCoralInProcess())
            .withPosition(2,1).withSize(2,1);
        //LED Color
        mainTab.addString("LED", () -> ledSubsystem.getLEDColor())
            .withPosition(2,2).withSize(2,1);

        //Algae - Distance
        // mainTab.addBoolean("isValidDistance", () -> algaeSubsystem.getIsValidRange())
        //     .withPosition(6,0).withSize(2,1);

        // mainTab.addDouble("distance Sensor", () -> algaeSubsystem.getDistanceSensor())
        //     .withPosition(6,1).withSize(2,1);
            

    }

 
    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-xboxDrive.getLeftY() * -xboxDrive.getLeftY()* Math.signum(xboxDrive.getLeftY()) * MaxSpeed/(xboxDrive.rightTrigger().getAsBoolean() ? 1.5 : 6)) // Drive forward with negative Y (forward)
                    .withVelocityY(-xboxDrive.getLeftX() * -xboxDrive.getLeftX()* Math.signum(xboxDrive.getLeftX()) * MaxSpeed/(xboxDrive.rightTrigger().getAsBoolean() ? 1.5 : 6)) // Drive left with negative X (left)
                    .withRotationalRate(-xboxDrive.getRightX() * MaxAngularRate/(xboxDrive.rightTrigger().getAsBoolean() ? 1 : 2)) // Drive counterclockwise with negative X (left)
            )
        );
        


// In RobotContainer.java, replace the vision alignment binding with:

xboxDrive.x().whileTrue(
    drivetrain.applyRequest(() -> visDrive
        // Forward/backward movement based on distance from target
        .withVelocityX(-visionSubsystem.calculateXPower(
            -xboxDrive.getLeftY() * MaxSpeed / 6,
            Constants.VisionConstants.xOffset,
            true) * MaxSpeed)
            
        // Left/right movement to center with the target
        .withVelocityY(-visionSubsystem.calculateYPower(
            -xboxDrive.getLeftX() * MaxSpeed / 6,
            Constants.VisionConstants.yOffsetLeft,
            true) * MaxSpeed)
            
        // Rotation to align parallel to the target
        .withRotationalRate(-visionSubsystem.calculateParallelRotationPower(
            -xboxDrive.getRightX() * MaxAngularRate / 2,
            true) * 0)
    )
);

        xboxDrive.b().onTrue(new InstantCommand(() -> ledSubsystem.changeAnimation(LEDSubsystem.AnimationTypes.Fire)));

        gamepadManipulator.leftTrigger().whileTrue(new AlgaeCommand(algaeSubsystem, AlgaeMode.EJECT));
        xboxDrive.leftTrigger().whileTrue(new AlgaeCommand(algaeSubsystem, AlgaeMode.INTAKE));
        xboxDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //xboxDrive.b().whileTrue(drivetrain.applyRequest(() ->

        //point.withModuleDirection(new Rotation2d(-xboxDrive.getLeftY(), -xboxDrive.getLeftX()))
        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xboxDrive.back().and(xboxDrive.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        xboxDrive.back().and(xboxDrive.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        xboxDrive.start().and(xboxDrive.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        xboxDrive.start().and(xboxDrive.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on a button press
        xboxDrive.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Algae Control - Driver controls intake, manipulator controls eject 
        xboxDrive.leftTrigger().whileTrue(new AlgaeCommand(algaeSubsystem, AlgaeMode.INTAKE));
        gamepadManipulator.leftTrigger().whileTrue(new AlgaeCommand(algaeSubsystem, AlgaeMode.EJECT));

       

        // Elevator Controls
        gamepadManipulator.a().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.INTAKE_POSITION));    // Intake
        gamepadManipulator.b().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.L1_POSITION));        // L1
        gamepadManipulator.x().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.L2_POSITION));        // L2
        gamepadManipulator.rightBumper().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Algae1));        // L3
        gamepadManipulator.leftBumper().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Algae2));        // L3
        
        gamepadManipulator.y().whileTrue(new InstantCommand(() -> coralHandler.nudge()))
        .onFalse(new InstantCommand(() -> coralHandler.stopCoralHandler()));
        // Elevator Emergency Stop
        gamepadManipulator.back().onTrue(new InstantCommand(() -> elevatorSubsystem.stop()));

        //Eject Coral
        gamepadManipulator.rightTrigger().whileTrue(new InstantCommand(() -> coralHandler.startOuttake()))
                    .onFalse(new InstantCommand(() -> coralHandler.stopCoralHandler()));

        gamepadManipulator.start().whileTrue(new InstantCommand(() -> coralHandler.back()))
          .onFalse(new InstantCommand(() -> coralHandler.stopCoralHandler()));

        // Manual Adjustments for Elevator
        gamepadManipulator.povUp().onTrue(new ElevatorCommand(elevatorSubsystem, true));     // Manual Up
        gamepadManipulator.povDown().onTrue(new ElevatorCommand(elevatorSubsystem, false));  // Manual Down

        

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    private void configureAutoChooser() {
           
       // NamedCommands.registerCommand("dropCoral", Commands.runOnce(()->{AutoCoralReleaseCommand};
        //NamedCommands.registerCommand("getAlgae", new AutoSubsystem.ReefProcessor(algaeSubsystem));
   //NamedCommands.registerCommand("ScoreCoral", new RunCommand(() -> AutoSubsystem.AutoExchange(coralHandler, elevatorSubsystem , algaeSubsystem)));

        // Set default option
        autoChooser.setDefaultOption("No Auto", new InstantCommand());

        // Add PathPlanner paths
        autoChooser.addOption("Go Offline", AutoSubsystem.getAutoCommand(AutoSubsystem.AutoMode.goOffline));
        autoChooser.addOption("IdealAuto", AutoSubsystem.getAutoCommand(AutoSubsystem.AutoMode.IdealAuto));
       // autoChooser.addOption("Reef Processor", AutoSubsystem.getAutoCommand(AutoSubsystem.AutoMode.ReefProcessor));
        //autoChooser.addOption("OL - CL2", AutoSubsystem.getAutoCommand(AutoSubsystem.AutoMode.OL_CL2));
        
        
          // Display on SmartDashboard
      Shuffleboard.getTab("Autonomous").add(autoChooser);
        }
    public Command getAutonomousCommand() {
        if (autoChooser.getSelected() != null){
            return autoChooser.getSelected();
        } else {
            return Commands.print("No autonomous command configured, if a path was chosen, this is an error.");
        }
    }

    
}
