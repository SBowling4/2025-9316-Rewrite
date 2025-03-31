package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.CoralHandlerSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;

@SuppressWarnings("unused")
public class Constants {

    public static final class ElevatorConstants {
        public static final double ELEVATOR_UP_POSITION = 0.25; // Units? (e.g., percentage, encoder ticks)
        public static final double ELEVATOR_DOWN_POSITION = 0; // Units?
        public static final int ELEVATOR_MOTOR_1_ID = 10; // Motor ID
        public static final int ELEVATOR_MOTOR_2_ID = 9;  // Motor ID
        public static final int ELEVATOR_ENCODER_PORT = 9; // Encoder port
        public static final int smartCurrentLimit = 60;

        public static final class ElevatorLeft {
            public static final int kFollowerModeLeaderId = 9;
            public static final boolean kFollowerModeIsInverted = true;
        }

        public static final class ElevatorRight { 
            public static final int kCanID = 9;
            public static final int kInputMode = 1;
            public static final int kIdleMode = 1;
            public static final boolean kInverted = false;
            
        }
    }


  public static final class CoralHandlerConstants {

        public static final int CORAL_HANDLER_MOTOR_ID = 11; // Coral Intake
        /*

         See Wiring Spreadsheet for more details:

        https://docs.google.com/spreadsheets/d/1y8MNmf4Ztvmj5xiLHOoS9DUJrCoks60W2GWf0nozFus/edit?gid=2021393044#gid=2021393044
         
        */
        public static final int HOPPER_BEAM_BREAK_ID = 0; // BEAM1 - Hopper Beam Break
        public static final int INTAKE_BEAM_BREAK_ID = 1; // BEAM2 - Inside Intake Beam Break
        public static final int OUTTAKE_BEAM_BREAK_ID = 2; // BEAM3 - Outside Intake Beam Break
        
    }

    public class AlgaeConstants {
        public static final int MOTOR_ID = 12;
        public static final Port DISTANCE_SENSOR_PORT = edu.wpi.first.wpilibj.I2C.Port.kOnboard;

    }

    

    public static final class VisionConstants {
        public static final String TARGET_CAMERA = "Camera";
        public static final double CAMERA_HEIGHT = 0.0; // Units? (e.g., meters)
        public static final double APRILTAG_RED_SHOOTER_HEIGHT = 0.0; // Units?
        public static final double CAMERA_PITCH = 0.0; // Units? (e.g., degrees)

        public static final double xOffset = .225; //0.25 -Foward +Back
        public static final double yOffsetLeft = 3; //.12 +LEFT -RIGHT
        
        
        
        
        
        
       //ignore 
        public static final double yOffsetRight = -.1;

        
        public static final Transform3d CAMERA_OFFSET = null; // Units?
		//Vision Rotation PID vars
        public static double VR_Kp = 0.00035;
        public static double VR_Ki = 0.0001;
        public static double VR_Kd = 0.0;

        //Vision Drive PID vars Left and Right
        public static double VY_Kp = 0.007;
        public static double VY_Ki = 0.00001;
        public static double VY_Kd = 0.00001;

        //Vision Drive PID var Forward and Back
        public static double VX_Kp = 0.275;
        public static double VX_Ki = 0.0;
        public static double VX_Kd = 0.0;
    }

    public static final class LEDConstants {
        public static final int[] RED = {255, 0, 0}; // RGB values
        public static final int[] YELLOW = {245, 239, 66}; // RGB values
        public static final int[] WHITE = {255, 255, 255}; // RGB values
        public static final int[] ORANGE = {255, 128, 0}; // RGB values
        public static final int[] GREEN = {60, 255, 0}; // RGB values
        public static final int[] BLUE = {20, 10, 255}; // RGB values
        public static final int CANDLE_ID = 35; // CAN ID
        public static final int NUM_OF_LEDS = 252; //Number of LEDs
        public static final int MAX_BRIGHTNESS_ANGLE = 90; // Units? (e.g., degrees)
        public static final int MID_BRIGHTNESS_ANGLE = 180; // Units?
        public static final int ZERO_BRIGHTNESS_ANGLE = 270; // Units?
       //TODO: These don't belong to LEDs
        public static final int VBAT_BUTTON = XboxController.Button.kA.value; // Xbox Button Value
        public static final int V5_BUTTON = XboxController.Button.kB.value; // Xbox Button Value
        public static final int CURRENT_BUTTON = XboxController.Button.kX.value; // Xbox Button Value
        public static final int TEMPERATURE_BUTTON = XboxController.Button.kY.value; // Xbox Button Value
    }
}