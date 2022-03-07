package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.components.Pigeon;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotMap.CAN;
import frc.robot.constants.RobotMap.PWM;
import frc.robot.utilities.MotorConfig;
import frc.robot.utilities.pid.PIDFTalonFX;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class RobotComponents {
    protected static class LEDComponents {
        public static final Spark CONTROLLER = new Spark(PWM.LED.CONTROLLER_PORT);
    }

    protected static class SwerveComponents {
        public static final double DRIVE_MOTOR_RAMP_RATE = 0.1;
        private static final TrigonTalonSRX PIGEON_SRX = new TrigonTalonSRX(CAN.Swerve.PIGEON_ID);
        public static final Pigeon PIGEON = new Pigeon(PIGEON_SRX);
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
                withOpenLoopRampRate(DRIVE_MOTOR_RAMP_RATE).
                withClosedLoopRampRate(DRIVE_MOTOR_RAMP_RATE).
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

    protected static class ShooterComponents {
        private static final MotorConfig LEFT_MOTOR_CONFIG = new MotorConfig()
                .withPID(RobotConstants.LOCAL_CONSTANTS.localShooterConstants.pidfCoefs)
                .withOpenLoopRampRate(0.5)
                .withClosedLoopRampRate(0.5)
                .coast()
                .inverted(false);
        public static final PIDFTalonSRX LEFT_MOTOR = new PIDFTalonSRX(
                CAN.Shooter.LEFT_MOTOR_ID, LEFT_MOTOR_CONFIG, ControlMode.Velocity);
        // Inverted because mechanically inverted
        private static final MotorConfig RIGHT_MOTOR_CONFIG = new MotorConfig(LEFT_MOTOR_CONFIG)
                .inverted(!LEFT_MOTOR_CONFIG.isInverted());
        public static final PIDFTalonSRX RIGHT_MOTOR = new PIDFTalonSRX(
                CAN.Shooter.RIGHT_MOTOR_ID, RIGHT_MOTOR_CONFIG, ControlMode.Velocity);
    }

    protected static class ClimberComponents {
        private static final MotorConfig MOTOR_CONFIG = new MotorConfig().
                brake().
                inverted(false).
                withOpenLoopRampRate(0.4).
                withClosedLoopRampRate(0.4).
                withPID(RobotConstants.LOCAL_CONSTANTS.localClimberConstants.pidfCoefs);
        public static final PIDFTalonFX LEFT_MOTOR = new PIDFTalonFX(
                CAN.Climber.LEFT_MOTOR_ID, MOTOR_CONFIG, ControlMode.Velocity);
        public static final PIDFTalonFX RIGHT_MOTOR = new PIDFTalonFX(
                CAN.Climber.RIGHT_MOTOR_ID, MOTOR_CONFIG, ControlMode.Velocity);
    }

    protected static class TransporterComponents {
        public static final TrigonTalonSRX MOTOR = SwerveComponents.FrontRight.ANGLE_ENCODER;
        public static final ColorSensorV3 COLOR_SENSOR = new ColorSensorV3(I2C.Port.kOnboard);
    }

    protected static class IntakeComponents {
        public static TrigonTalonSRX MOTOR = SwerveComponents.FrontLeft.ANGLE_ENCODER;
    }

    protected static class IntakeOpenerComponents {
        private static final MotorConfig MOTOR_CONFIG = new MotorConfig().
                coast().
                inverted(true).
                withOpenLoopRampRate(0.5).
                withClosedLoopRampRate(0.5).
                withPID(RobotConstants.LOCAL_CONSTANTS.localIntakeOpenerConstants.pidfCoefs);
        public static PIDFTalonSRX MOTOR = new PIDFTalonSRX(
                CAN.IntakeOpener.MOTOR_ID, MOTOR_CONFIG, ControlMode.Position);
    }

    protected static class PitcherComponents {
        private static final MotorConfig MOTOR_CONFIG = new MotorConfig().
                brake().
                inverted(false).
                withOpenLoopRampRate(0.4).
                withClosedLoopRampRate(0.4).
                withPID(RobotConstants.LOCAL_CONSTANTS.localPitcherConstants.pidfCoefs);
        public static PIDFTalonSRX MOTOR = new PIDFTalonSRX(CAN.Pitcher.MOTOR_ID, MOTOR_CONFIG, ControlMode.Position);
    }

    protected static class LoaderComponents {
        private static final MotorConfig MOTOR_CONFIG = new MotorConfig().
                brake().
                withOpenLoopRampRate(0.5).
                withClosedLoopRampRate(0.5);
        public static final PIDFTalonSRX MOTOR = new PIDFTalonSRX(
                CAN.Loader.MOTOR_ID, MOTOR_CONFIG, ControlMode.Velocity);
    }
}
