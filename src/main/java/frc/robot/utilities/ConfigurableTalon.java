package frc.robot.utilities;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.constants.RobotConstants;

public interface ConfigurableTalon extends IMotorController {
    default void config(MotorConfig motorConfig) {
        setInverted(motorConfig.isInverted());
        setSensorPhase(motorConfig.isSensorInverted());
        setNeutralMode(motorConfig.getNeutralMode());
        enableVoltageCompensation(motorConfig.getVoltageCompSaturation() > 0);
        ce_configFeedbackNotContinuous(motorConfig.isFeedbackNotContinuous());
        ce_configOpenloopRamp(motorConfig.getOpenLoopRampRate());
        ce_configClosedloopRamp(motorConfig.getClosedLoopRampRate());
        ce_configVoltageCompSaturation(motorConfig.getVoltageCompSaturation());
        ce_configSupplyCurrentLimit(motorConfig.getCurrentLimitConfig());
        ce_configSelectedFeedbackSensor(motorConfig.getFeedbackDevice());
        ce_configRemoteFeedbackFilter(
                motorConfig.getRemoteSensorSourceDeviceId(), motorConfig.getRemoteSensorSourceType());
        ce_config_kP(0, motorConfig.getCoefs().getKP());
        ce_config_kI(0, motorConfig.getCoefs().getKI());
        ce_config_kD(0, motorConfig.getCoefs().getKD());
        ce_config_kF(0, motorConfig.getCoefs().getKV());
        ce_configAllowableClosedloopError(0, (int) motorConfig.getCoefs().getTolerance());
    }

    ErrorCode configFactoryDefault(int timeoutMs);

    default ErrorCode ce_configFactoryDefault() {
        return CTREUtil.checkError(() -> configFactoryDefault(RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configOpenloopRamp(double seconds) {
        return CTREUtil.checkError(() -> configOpenloopRamp(seconds, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configClosedloopRamp(double seconds) {
        return CTREUtil.checkError(() -> configClosedloopRamp(seconds, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configVoltageCompSaturation(double volts) {
        return CTREUtil.checkError(() -> configVoltageCompSaturation(volts, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration config, int timeoutMs);

    default ErrorCode ce_configSupplyCurrentLimit(SupplyCurrentLimitConfiguration config) {
        return CTREUtil.checkError(() -> configSupplyCurrentLimit(config, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_config_kP(int slotIdx, double value) {
        return CTREUtil.checkError(() -> config_kP(slotIdx, value, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_config_kI(int slotIdx, double value) {
        return CTREUtil.checkError(() -> config_kI(slotIdx, value, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_config_kD(int slotIdx, double value) {
        return CTREUtil.checkError(() -> config_kD(slotIdx, value, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_config_kF(int slotIdx, double value) {
        return CTREUtil.checkError(() -> config_kF(slotIdx, value, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configAllowableClosedloopError(int slotIdx, int allowableCloseLoopError) {
        return CTREUtil.checkError(
                () -> configAllowableClosedloopError(
                        slotIdx, allowableCloseLoopError, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configRemoteFeedbackFilter(
            int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal) {
        return CTREUtil.checkError(() -> configRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal,
                RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx) {
        return CTREUtil.checkError(
                () -> configSelectedFeedbackSensor(
                        RemoteFeedbackDevice.valueOf(feedbackDevice.value), pidIdx,
                        RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configSelectedFeedbackSensor(FeedbackDevice feedbackDevice) {
        return ce_configSelectedFeedbackSensor(feedbackDevice, 0);
    }

    default ErrorCode ce_setSelectedSensorPosition(int sensorPos, int pidIdx) {
        return CTREUtil.checkError(
                () -> setSelectedSensorPosition(sensorPos, pidIdx, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_setSelectedSensorPosition(int sensorPos) {
        return ce_setSelectedSensorPosition(sensorPos, 0);
    }

    ErrorCode configFeedbackNotContinuous(boolean feedbackNotContinuous, int timeoutMs);

    default ErrorCode ce_configFeedbackNotContinuous(boolean feedbackNotContinuous) {
        return CTREUtil.checkError(
                () -> configFeedbackNotContinuous(feedbackNotContinuous, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }

    default ErrorCode ce_configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource) {
        return ce_configRemoteFeedbackFilter(deviceID, remoteSensorSource, 0);
    }
}
