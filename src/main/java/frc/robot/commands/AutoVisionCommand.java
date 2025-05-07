package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class AutoVisionCommand  extends Command{
    private boolean vision = false;
    private final VisionSubsystem visionSubsystem;
    private final CoralHandlerSubsystem coralHandlerSubsystem;
    public final CommandSwerveDrivetrain drivetrain;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.RobotCentric visDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public AutoVisionCommand(CoralHandlerSubsystem coralHandlerSubsystem, VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain, boolean vision){
        this.visionSubsystem = visionSubsystem;
        this.coralHandlerSubsystem = coralHandlerSubsystem;
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(visionSubsystem , drivetrain);
    }


    @Override
    public void execute() {
        if (!visionSubsystem.hasTarget()) {
            System.out.println("AutoVisionCommand: No target detected.");
            SwerveRequest.RobotCentric request = visDrive
                .withVelocityX(0.00)
                .withVelocityY(0.00)
                .withRotationalRate(0);
            drivetrain.setControl(request);
            return;
        }   
        System.out.println("AutoVisionCommand: target detected.");

        // Calculate the PID outputs
        double xPower = visionSubsystem.calculateXPower(0, Constants.VisionConstants.xOffset, this.vision);
        double yPower = visionSubsystem.calculateYPower(0, Constants.VisionConstants.yOffsetLeft, this.vision);
        double rotationPower = visionSubsystem.calculateParallelRotationPower(0, this.vision);
       
        // Compute final velocities
        double velocityX = -xPower * MaxSpeed;
        double velocityY = -yPower * MaxSpeed;
        double rotationalRate = -rotationPower * MaxAngularRate;
       
        // Debug prints to verify computed values
        System.out.println("Calculated xPower: " + xPower + ", velocityX: " + velocityX);
        System.out.println("Calculated yPower: " + yPower + ", velocityY: " + velocityY);
        System.out.println("Calculated rotationPower: " + rotationPower + ", rotationalRate: " + rotationalRate);
        System.out.println("Current range: " + visionSubsystem.getRange().orElse(-1.0));
       
        // Apply the drive command
        // Create the request
        SwerveRequest.RobotCentric request = visDrive
        .withVelocityX(velocityX)
        .withVelocityY(velocityY*1.5)
        .withRotationalRate(rotationalRate);

        System.out.println(request);

        // Apply the request
        drivetrain.setControl(request);

        /*drivetrain.applyRequest(() -> visDrive
        // Forward/backward movement based on distance from target
        .withVelocityX(-visionSubsystem.calculateXPower(
           0,
            0.26,
            true) * MaxSpeed)
            
        // Left/right movement to center with the target
        .withVelocityY(visionSubsystem.calculateYPower(
            0,
            -10,
            true) * MaxSpeed)
            
        // Rotation to align parallel to the target
        .withRotationalRate(visionSubsystem.calculateParallelRotationPower(
            0,
            true) * MaxAngularRate)
    ); */
    }
    public boolean isFinished() {
        
        double range = visionSubsystem.getRange().orElse(2.0);
        if(range < .325){
            SwerveRequest.RobotCentric request = visDrive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0);
            drivetrain.setControl(request);
        }
        return range < 0.325;
        //return visionSubsystem.getRange().orElse(0.0) < 0.5;
    }

}
