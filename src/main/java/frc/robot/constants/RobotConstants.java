package frc.robot.constants;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.components.Pigeon;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotComponents.*;
import frc.robot.utilities.JsonHandler;
import frc.robot.utilities.Module;
import frc.robot.utilities.pid.PIDCoefs;
import frc.robot.utilities.pid.PIDFTalonFX;
import frc.robot.utilities.pid.PIDFTalonSRX;

/**
 * All the constants to be uses for the robot
 */
public class RobotConstants {
    public static final int DEFAULT_CAN_TIMEOUT = 30;
    protected static final LocalConstants LOCAL_CONSTANTS = JsonHandler.getConstants();

    public static class LimelightConstants {
        public static final double DISTANCE_CALCULATION_A_COEFFICIENT = 1;
        public static final double DISTANCE_CALCULATION_B_COEFFICIENT = 1;
        public static final double DISTANCE_CALCULATION_C_COEFFICIENT = 1;
        public static final double LIMELIGHT_ANGLE_OFFSET = 1;
        public static final double LIMELIGHT_OFFSET_X = 1;
        public static final double LIMELIGHT_OFFSET_Y = 1;
        public static final String DEFAULT_TABLE_KEY = "limelight";
    }

    public static class TesterConstants {
        public static final int SECONDS_TO_WAIT = 1;
        public static final double DEFAULT_MOVE_POWER = 3;
        public static final int LED_BLINK_AMOUNT = 10;
    }

    public static class VisionConstants {
        public static final PIDCoefs ROTATION_SETTINGS = new PIDCoefs(0, 0, 0, 0, 0);
        public static final double TARGET_TIME_OUT = 0.1;
    }

    public static class SwerveConstants {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-
        public static final Pigeon PIGEON = SwerveComponents.PIGEON;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.29765 * 2;
        public static final double WHEEL_BASE = 0.29765 * 2;
        public static final double WHEEL_DIAMETER = 0.1;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double DRIVE_GEAR_RATIO = (8.14 / 1.0);
        public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        public static final double MAX_SPEED = 4.5; //meters per second
        public static final double MAX_ANGULAR_VELOCITY = 11.5;

        public static final SwerveModuleConstants FRONT_LEFT_CONSTANTS = new SwerveModuleConstants(
                SwerveComponents.FrontLeft.ANGLE_MOTOR,
                SwerveComponents.FrontLeft.DRIVE_MOTOR,
                SwerveComponents.FrontLeft.ANGLE_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.frontLeftModuleConstants,
                Module.FRONT_LEFT
        );

        public static final SwerveModuleConstants FRONT_RIGHT_CONSTANTS = new SwerveModuleConstants(
                SwerveComponents.FrontRight.ANGLE_MOTOR,
                SwerveComponents.FrontRight.DRIVE_MOTOR,
                SwerveComponents.FrontRight.ANGLE_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.frontRightModuleConstants,
                Module.FRONT_RIGHT
        );

        public static final SwerveModuleConstants REAR_LEFT_CONSTANTS = new SwerveModuleConstants(
                SwerveComponents.RearLeft.ANGLE_MOTOR,
                SwerveComponents.RearLeft.DRIVE_MOTOR,
                SwerveComponents.RearLeft.ANGLE_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.rearLeftModuleConstants,
                Module.REAR_LEFT
        );

        public static final SwerveModuleConstants REAR_RIGHT_CONSTANTS = new SwerveModuleConstants(
                SwerveComponents.RearRight.ANGLE_MOTOR,
                SwerveComponents.RearRight.DRIVE_MOTOR,
                SwerveComponents.RearRight.ANGLE_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.rearRightModuleConstants,
                Module.REAR_RIGHT
        );
    }

    public static class LedConstants {
        public static final Spark CONTROLLER = LEDComponents.CONTROLLER;
    }

    public static class DriverConstants {
        public static final boolean SQUARED_CONTROLLER_DRIVING = true;
        public static final double DRIVING_SPEED_DIVIDER = LOCAL_CONSTANTS.localDriverConstants.drivingSpeedDivider;
        public static final int XBOX_PORT = 0;
        public static final double RUMBLE_INTERMISSION_TIME = 0.15;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    public static class ShooterConstants {
        public static final PIDFTalonSRX LEFT_MOTOR = ShooterComponents.LEFT_MOTOR;
        public static final PIDFTalonSRX RIGHT_MOTOR = ShooterComponents.RIGHT_MOTOR;
    }

    public static class ClimberConstants {
        public static final PIDFTalonFX LEFT_MOTOR = ClimberComponents.LEFT_MOTOR;
        public static final PIDFTalonFX RIGHT_MOTOR = ClimberComponents.RIGHT_MOTOR;
        public static final int MAX_POSITION = 7000; // in ticks
    }

    public static class TransporterConstants {
        public static final TrigonTalonSRX MOTOR = TransporterComponents.MOTOR;
        public static final ColorSensorV3 COLOR_SENSOR = TransporterComponents.COLOR_SENSOR;
        //TODO: set to real value
        public static final int STALL_CURRENT_LIMIT = 20;
    }

    public static class IntakeConstants {
        public static final TrigonTalonSRX MOTOR = IntakeComponents.MOTOR;
        public static double STALL_CURRENT_LIMIT = 20;
    }

    public static class IntakeOpenerConstants {
        public static final PIDFTalonSRX MOTOR = IntakeOpenerComponents.MOTOR;
        public static final double GEAR_RATIO = 81;
        public static double OPENED_ANGLE = 97; // in degrees
        public static double STALL_CURRENT_LIMIT = 20;
    }

    public static class PitcherConstants {
        public static final TrigonTalonSRX MOTOR = PitcherComponents.MOTOR;
        public static final double GEAR_RATIO = 10;
        public static double OPEN_ANGLE = 50; // in degrees
        public static double CLOSED_ANGLE = 70; // in degrees
    }

    public static class LoaderConstants {
        public static final PIDFTalonSRX MOTOR = LoaderComponents.MOTOR;
    }

    /**
     * Writes the LOCAL_CONSTANTS values to the json file.
     */
    public static void write() {
        JsonHandler.write(LOCAL_CONSTANTS);
    }
}