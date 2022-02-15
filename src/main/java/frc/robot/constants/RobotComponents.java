package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.components.Pigeon;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotMap.CAN;
import frc.robot.constants.RobotMap.PWM;
import frc.robot.utilities.MotorConfig;
import frc.robot.utilities.pid.PIDFTalonFX;

public class RobotComponents {
    protected static class LEDComponents {
        public static final Spark CONTROLLER = new Spark(PWM.LED.CONTROLLER_PORT);
    }

    protected static class SwerveComponents {
        public static final Pigeon PIGEON = new Pigeon(new TrigonTalonSRX(12));

        // configs
        private static final MotorConfig ANGLE_MOTOR_CONFIG = new MotorConfig().
                inverted(false).
                sensorPhase(false).
                withOpenLoopRampRate(2).
                withClosedLoopRampRate(0.5).
                withPrimaryFeedbackDevice(FeedbackDevice.IntegratedSensor).
                withFeedbackNotContinuous(true).
                brake().
                withCurrentLimit(new SupplyCurrentLimitConfiguration(
                        true,
                        25,
                        40,
                        0.1
                ));
        private static final MotorConfig DRIVE_MOTOR_CONFIG = new MotorConfig().
                inverted(true).
                sensorPhase(false).
                withOpenLoopRampRate(0.1).
                withClosedLoopRampRate(0.1).
                brake().
                withCurrentLimit(new SupplyCurrentLimitConfiguration(
                        true,
                        34,
                        60,
                        0.1
                ));
        private static final MotorConfig ANGLE_ENCODER_CONFIG = new MotorConfig().
                withFeedbackNotContinuous(true).
                withPrimaryFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Absolute);

        private static final LocalConstants.LocalSwerveConstants.LocalSwerveModules LOCAL_SWERVE_MODULES_CONSTANTS =
                RobotConstants.LOCAL_CONSTANTS.localSwerveConstants.modules;

        public static class FrontLeft {
            public static final TrigonTalonSRX ANGLE_ENCODER = new TrigonTalonSRX(
                    CAN.Swerve.FRONT_LEFT_ANGLE_ENCODER_ID, ANGLE_ENCODER_CONFIG);
            public static final PIDFTalonFX ANGLE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.FRONT_LEFT_ANGLE_MOTOR_ID, ANGLE_MOTOR_CONFIG.withPID(
                            LOCAL_SWERVE_MODULES_CONSTANTS.frontLeftModuleConstants.angleCoefs).
                    withRemoteSensorSource(
                            CAN.Swerve.FRONT_LEFT_ANGLE_ENCODER_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0).
                    withSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor0),
                    ControlMode.Position);
            public static final PIDFTalonFX DRIVE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.FRONT_LEFT_DRIVE_MOTOR_ID, DRIVE_MOTOR_CONFIG.withPID(
                    LOCAL_SWERVE_MODULES_CONSTANTS.frontLeftModuleConstants.driveCoefs),
                    ControlMode.Velocity);
        }

        public static class FrontRight {
            public static final TrigonTalonSRX ANGLE_ENCODER = new TrigonTalonSRX(
                    CAN.Swerve.FRONT_RIGHT_ANGLE_ENCODER_ID, ANGLE_ENCODER_CONFIG);
            public static final PIDFTalonFX ANGLE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.FRONT_RIGHT_ANGLE_MOTOR_ID, ANGLE_MOTOR_CONFIG.withPID(
                            LOCAL_SWERVE_MODULES_CONSTANTS.frontRightModuleConstants.angleCoefs).
                    withRemoteSensorSource(
                            CAN.Swerve.FRONT_RIGHT_ANGLE_ENCODER_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0).
                    withSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor0),
                    ControlMode.Position);
            public static final PIDFTalonFX DRIVE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.FRONT_RIGHT_DRIVE_MOTOR_ID, DRIVE_MOTOR_CONFIG.withPID(
                    LOCAL_SWERVE_MODULES_CONSTANTS.frontRightModuleConstants.driveCoefs),
                    ControlMode.Velocity);
        }

        public static class RearLeft {
            public static final TrigonTalonSRX ANGLE_ENCODER = new TrigonTalonSRX(
                    CAN.Swerve.REAR_LEFT_ANGLE_ENCODER_ID, ANGLE_ENCODER_CONFIG);
            public static final PIDFTalonFX ANGLE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.REAR_LEFT_ANGLE_MOTOR_ID, ANGLE_MOTOR_CONFIG.withPID(
                            LOCAL_SWERVE_MODULES_CONSTANTS.rearLeftModuleConstants.angleCoefs).
                    withRemoteSensorSource(
                            CAN.Swerve.REAR_LEFT_ANGLE_ENCODER_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0).
                    withSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor0),
                    ControlMode.Position);
            public static final PIDFTalonFX DRIVE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.REAR_LEFT_DRIVE_MOTOR_ID, DRIVE_MOTOR_CONFIG.withPID(
                    LOCAL_SWERVE_MODULES_CONSTANTS.rearLeftModuleConstants.driveCoefs),
                    ControlMode.Velocity);
        }

        public static class RearRight {
            public static final TrigonTalonSRX ANGLE_ENCODER = new TrigonTalonSRX(
                    CAN.Swerve.REAR_RIGHT_ANGLE_ENCODER_ID, ANGLE_ENCODER_CONFIG);
            public static final PIDFTalonFX ANGLE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.REAR_RIGHT_ANGLE_MOTOR_ID, ANGLE_MOTOR_CONFIG.withPID(
                            LOCAL_SWERVE_MODULES_CONSTANTS.rearRightModuleConstants.angleCoefs).
                    withRemoteSensorSource(
                            CAN.Swerve.REAR_RIGHT_ANGLE_ENCODER_ID, RemoteSensorSource.TalonSRX_SelectedSensor, 0).
                    withSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor0),
                    ControlMode.Position);
            public static final PIDFTalonFX DRIVE_MOTOR = new PIDFTalonFX(
                    CAN.Swerve.REAR_RIGHT_DRIVE_MOTOR_ID, DRIVE_MOTOR_CONFIG.withPID(
                    LOCAL_SWERVE_MODULES_CONSTANTS.rearRightModuleConstants.driveCoefs),
                    ControlMode.Velocity);
        }
    }
}
