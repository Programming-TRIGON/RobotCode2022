package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.components.Pigeon;
import frc.robot.constants.RobotComponents.LEDComponents;
import frc.robot.constants.RobotComponents.SwerveComponents;
import frc.robot.utilities.JsonHandler;
import frc.robot.utilities.pid.PIDCoefs;

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
                SwerveComponents.FRONT_LEFT_ANGLE_MOTOR,
                SwerveComponents.FRONT_LEFT_DRIVE_MOTOR,
                SwerveComponents.FRONT_LEFT_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.frontLeftModuleConstants.encoderOffset
        );

        public static final SwerveModuleConstants FRONT_RIGHT_CONSTANTS = new SwerveModuleConstants(
                SwerveComponents.FRONT_RIGHT_ANGLE_MOTOR,
                SwerveComponents.FRONT_RIGHT_DRIVE_MOTOR,
                SwerveComponents.FRONT_RIGHT_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.frontRightModuleConstants.encoderOffset
        );

        public static final SwerveModuleConstants REAR_LEFT_CONSTANTS = new SwerveModuleConstants(
                SwerveComponents.REAR_LEFT_ANGLE_MOTOR,
                SwerveComponents.REAR_LEFT_DRIVE_MOTOR,
                SwerveComponents.REAR_LEFT_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.rearLeftModuleConstants.encoderOffset
        );

        public static final SwerveModuleConstants REAR_RIGHT_CONSTANTS = new SwerveModuleConstants(
                SwerveComponents.REAR_RIGHT_ANGLE_MOTOR,
                SwerveComponents.REAR_RIGHT_DRIVE_MOTOR,
                SwerveComponents.REAR_RIGHT_ENCODER,
                LOCAL_CONSTANTS.localSwerveConstants.modules.rearRightModuleConstants.encoderOffset
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
    }
}
