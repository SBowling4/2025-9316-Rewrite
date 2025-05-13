package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.constants.TunerConstants;
import frc.robot.util.constants.Constants.VisionConstants;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DataLogManager;

public class AutoAlign extends Command{
    private final VisionSubsystem visionSubsystem = RobotContainer.getVisionSubsystem();
    private final CommandSwerveDrivetrain drivetrain = RobotContainer.getDrivetrain();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.RobotCentric visDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public AutoAlign(){
        addRequirements(visionSubsystem, drivetrain);
    }


    @Override
    public void execute() {
        if (!visionSubsystem.hasTarget()) {
            DataLogManager.log("AutoAlign: No target detected.");
            SwerveRequest.RobotCentric request = visDrive
                .withVelocityX(0.00)
                .withVelocityY(0.00)
                .withRotationalRate(0);
            drivetrain.setControl(request);
            return;
        }   
        DataLogManager.log("AutoAlign: target detected.");

        // Calculate the PID outputs
        double xPower = visionSubsystem.calculateXPower(0, VisionConstants.xOffset, true);
        double yPower = visionSubsystem.calculateYPower(0, VisionConstants.yOffsetLeft);
        double rotationPower = visionSubsystem.calculateParallelRotationPower(0);
       
        // Compute final velocities
        double velocityX = -xPower * MaxSpeed;
        double velocityY = -yPower * MaxSpeed;
        double rotationalRate = -rotationPower * MaxAngularRate;
       
        // Debug prints to verify computed values
        DataLogManager.log("Calculated xPower: " + xPower + ", velocityX: " + velocityX);
        DataLogManager.log("Calculated yPower: " + yPower + ", velocityY: " + velocityY);
        DataLogManager.log("Calculated rotationPower: " + rotationPower + ", rotationalRate: " + rotationalRate);
        DataLogManager.log("Current range: " + visionSubsystem.getRange().orElse(-1.0));
       
        // Create the request
        SwerveRequest.RobotCentric request = visDrive
        .withVelocityX(velocityX)
        .withVelocityY(velocityY*1.5)
        .withRotationalRate(rotationalRate);

        DataLogManager.log(request.toString());

        // Apply the request
        drivetrain.setControl(request);
    }

    public boolean isFinished() {    
        Optional<Double> range = visionSubsystem.getRange();

        if (range.isEmpty()) return false;
        
        return range.get() < 0.325;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveRequest.RobotCentric request = visDrive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

        drivetrain.setControl(request);
    }

}
