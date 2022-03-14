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

    /**
     * Writes the LOCAL_CONSTANTS values to the json file.
     */
    public static void write() {
        JsonHandler.write(LOCAL_CONSTANTS);
    }

    public static class HubLimelightConstants {
        public static final double DISTANCE_CALCULATION_A_COEFFICIENT = 0.0023;
        public static final double DISTANCE_CALCULATION_B_COEFFICIENT = -0.0962;
        public static final double DISTANCE_CALCULATION_C_COEFFICIENT = 1.75;
    }

    public static class TesterConstants {
        public static final int SECONDS_TO_WAIT = 1;
        public static final double DEFAULT_MOVE_POWER = 3;
        public static final int LED_BLINK_AMOUNT = 10;
    }

    public static class VisionConstants {
        public static final PIDCoefs HUB_TTT_COEFS = LOCAL_CONSTANTS.localSwerveConstants.turnToTargetCoefs;
        public static final PIDCoefs DRIVE_SETTINGS = LOCAL_CONSTANTS.localSwerveConstants.driveToTargetCoefs;
        public static final double DISTANCE_POWER = 0.2;
        public static final double DISTANCE_TOLERANCE = 0.2;
        public static final double ANGLE_TOLERANCE = 0.15;
        public static final double TARGET_TIME_OUT = 0.1;
    }

    public static class SwerveConstants {
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-
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

        public static final CharacterizationConstants CHARACTERIZATION_CONSTANTS = new CharacterizationConstants(
                0.2, 0.1, 3, 2, 5, false);
    }

    public static class LedConstants {
        public static final Spark CONTROLLER = LEDComponents.CONTROLLER;
    }

    public static class DriverConstants {
        public static final boolean SQUARED_CONTROLLER_DRIVING = true;
        public static final double SPEED_DIVIDER = 1;
        public static final double ROTATION_SPEED_DIVIDER = 2;
        public static final double ENDGAME_SPEED_DIVIDER = 3;
        public static final double COLLECT_SPEED_DIVIDER = 2;
        public static final double COLLECT_ROTATION_SPEED_DIVIDER = 4;
        public static final double ENDGAME_ROTATION_SPEED_DIVIDER = 3;
        public static final int XBOX_PORT = 0;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    public static class CommanderConstants {
        public static final int XBOX_PORT = 1;
        public static final boolean SQUARED_CONTROLLER_DRIVING = true;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    public static class ShooterConstants {
        public static final PIDFTalonSRX LEFT_MOTOR = ShooterComponents.LEFT_MOTOR;
        public static final PIDFTalonSRX RIGHT_MOTOR = ShooterComponents.RIGHT_MOTOR;
        public static final CharacterizationConstants CHARACTERIZATION_CONSTANTS = new CharacterizationConstants(
                0.4, 0.2, 3, 5, 2.5, false);
        public static final double TRANSPORTER_WAIT_TIME = 0.2;
    }

    public static class ClimberConstants {
        public static final PIDFTalonFX LEFT_MOTOR = ClimberComponents.LEFT_MOTOR;
        public static final PIDFTalonFX RIGHT_MOTOR = ClimberComponents.RIGHT_MOTOR;
        public static final int MAX__RIGHT_POSITION = 106133; // in ticks
        public static final int MIN__RIGHT_POSITION = -102133; // in ticks
        public static final int MAX_LEFT_POSE = 154222; // in ticks
        public static final int MIN_LEFT_POSE = -159222; // in ticks
        public static final int POSITION_TOLERANCE = 10000; // in ticks
        public static final double TRIGGER_DEADBAND = 0.6;
        public static final double OVERRIDDEN_POWER = 0.5;
    }

    public static class TransporterConstants {
        public static final TrigonTalonSRX MOTOR = TransporterComponents.MOTOR;
        public static final ColorSensorV3 COLOR_SENSOR = TransporterComponents.COLOR_SENSOR;
        //TODO: set reasonable power
        public static final double POWER = 0.55;
    }

    public static class IntakeConstants {
        public static final TrigonTalonSRX MOTOR = IntakeComponents.MOTOR;
        public static final double POWER = 0.8;
    }

    public static class IntakeOpenerConstants {
        public static final TrigonTalonSRX MOTOR = IntakeOpenerComponents.MOTOR;
        public static final double OPENING_POWER = 0.3;
        public static final double CLOSING_POWER = 0.4;
    }

    public static class PitcherConstants {
        public static final PIDFTalonSRX MOTOR = PitcherComponents.MOTOR;
        public static final double GEAR_RATIO = 10 / 1;
        public static final double MAX_ANGLE = 20; // in degrees
        public static final LocalConstants.LocalPitcherConstants LOCAL_PITCHER_CONSTANTS =
                LOCAL_CONSTANTS.localPitcherConstants;
    }

    public static class LoaderConstants {
        public static final TrigonTalonSRX MOTOR = LoaderComponents.MOTOR;
        //TODO: set to correct power
        public static final double POWER = 0.4;
    }
}