package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.robot.utilities.pid.PIDFCoefs;

/**
 * This class stores the configuration of a motor
 */
public class MotorConfig {

    private double openLoopRampRate;
    private double closedLoopRampRate;
    private boolean isInverted;
    private boolean isSensorInverted;
    private boolean feedbackNotContinuous;
    private NeutralMode neutralMode;
    private double voltageCompSaturation;
    private SupplyCurrentLimitConfiguration currentLimitConfig;
    private FeedbackDevice primaryFeedbackDevice;
    private FeedbackDevice secondaryFeedbackDevice;
    private RemoteSensorSource remoteSensorSource0Type;
    private RemoteSensorSource remoteSensorSource1Type;
    private int remoteSensorSource0DeviceId;
    private int remoteSensorSource1DeviceId;
    private PIDFCoefs coefs;

    /**
     * Default constructor
     */
    public MotorConfig() {
        openLoopRampRate = 0;
        closedLoopRampRate = 0;
        isInverted = false;
        isSensorInverted = false;
        feedbackNotContinuous = false;
        neutralMode = NeutralMode.Coast;
        voltageCompSaturation = 0;
        currentLimitConfig = new SupplyCurrentLimitConfiguration();
        primaryFeedbackDevice = FeedbackDevice.IntegratedSensor;
        secondaryFeedbackDevice = FeedbackDevice.None;
        remoteSensorSource0Type = RemoteSensorSource.Off;
        remoteSensorSource1Type = RemoteSensorSource.Off;
        remoteSensorSource0DeviceId = 0;
        remoteSensorSource1DeviceId = 0;
        coefs = new PIDFCoefs();
    }

    /**
     * Copy constructor
     */
    public MotorConfig(MotorConfig config) {
        openLoopRampRate = config.getOpenLoopRampRate();
        closedLoopRampRate = config.getClosedLoopRampRate();
        isInverted = config.isInverted();
        isSensorInverted = config.isSensorInverted();
        feedbackNotContinuous = config.isFeedbackNotContinuous();
        neutralMode = config.getNeutralMode();
        voltageCompSaturation = config.getVoltageCompSaturation();
        currentLimitConfig = config.getCurrentLimitConfig();
        primaryFeedbackDevice = config.getPrimaryFeedbackDevice();
        secondaryFeedbackDevice = config.getSecondaryFeedbackDevice();
        remoteSensorSource0Type = config.getRemoteSensorSource0Type();
        remoteSensorSource1Type = config.getRemoteSensorSource1Type();
        remoteSensorSource0DeviceId = config.getRemoteSensorSource0DeviceId();
        remoteSensorSource1DeviceId = config.getRemoteSensorSource1DeviceId();
        coefs = config.getCoefs();
    }

    public double getOpenLoopRampRate() {
        return openLoopRampRate;
    }

    public double getClosedLoopRampRate() {
        return closedLoopRampRate;
    }

    public boolean isInverted() {
        return isInverted;
    }

    public boolean isSensorInverted() {
        return isSensorInverted;
    }

    public boolean isFeedbackNotContinuous() {
        return feedbackNotContinuous;
    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    public double getVoltageCompSaturation() {
        return voltageCompSaturation;
    }

    public SupplyCurrentLimitConfiguration getCurrentLimitConfig() {
        return currentLimitConfig;
    }

    public FeedbackDevice getPrimaryFeedbackDevice() {
        return primaryFeedbackDevice;
    }

    public FeedbackDevice getSecondaryFeedbackDevice() {
        return secondaryFeedbackDevice;
    }

    public RemoteSensorSource getRemoteSensorSource0Type() {
        return remoteSensorSource0Type;
    }

    public RemoteSensorSource getRemoteSensorSource1Type() {
        return remoteSensorSource1Type;
    }

    public int getRemoteSensorSource0DeviceId() {
        return remoteSensorSource0DeviceId;
    }

    public int getRemoteSensorSource1DeviceId() {
        return remoteSensorSource1DeviceId;
    }

    public PIDFCoefs getCoefs() {
        return coefs;
    }

    /**
     * Chain setter for the open loop ramp rate.
     *
     * @param openLoopRampRate Minimum desired time to go from
     *                         neutral to full throttle during open loop. A value of
     *                         '0' will disable the ramp.
     */
    public MotorConfig withOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
        return this;
    }

    /**
     * Chain setter for the PID coefficients.
     *
     * @param coefs The PID coefficients.
     */
    public MotorConfig withPID(PIDFCoefs coefs) {
        this.coefs = coefs;
        return this;
    }

    /**
     * Chain setter for the closed loop ramp rate.
     *
     * @param closedLoopRampRate Minimum desired time to go from
     *                           neutral to full throttle during closed loop. A value of
     *                           '0' will disable the ramp.
     */
    public MotorConfig withClosedLoopRampRate(double closedLoopRampRate) {
        this.closedLoopRampRate = closedLoopRampRate;
        return this;
    }

    /**
     * Chain setter for the neutral mode. Sets it to brake.
     */
    public MotorConfig brake() {
        this.neutralMode = NeutralMode.Brake;
        return this;
    }

    /**
     * Chain setter for the neutral mode. Sets it to coast.
     */
    public MotorConfig coast() {
        this.neutralMode = NeutralMode.Coast;
        return this;
    }

    /**
     * Chain setter for the voltage compensation saturation
     *
     * @param voltageCompSaturation This is the max voltage to apply to
     *                              the hbridge when voltage compensation
     *                              is enabled. For example, if 10 (volts)
     *                              is specified and a motor controller is
     *                              commanded to 0.5 (PercentOutput,
     *                              closed-loop, etc) then the motor
     *                              controller will attempt to apply a
     *                              duty-cycle to produce 5V. A value of
     *                              '0' disables this feature.
     */
    public MotorConfig withVoltageCompSaturation(double voltageCompSaturation) {
        this.voltageCompSaturation = voltageCompSaturation;
        return this;
    }

    /**
     * Chain setter for the supply current limit configuration
     *
     * @param currentLimitConfig Supply-side current limiting. This is
     *                           typically used to prevent breakers
     *                           from tripping.
     */
    public MotorConfig withCurrentLimit(SupplyCurrentLimitConfiguration currentLimitConfig) {
        this.currentLimitConfig = currentLimitConfig;
        return this;
    }

    /**
     * Chain setter for the sensor inversion
     *
     * @param isSensorInverted Whether the sensor is inverted
     */
    public MotorConfig sensorPhase(boolean isSensorInverted) {
        this.isSensorInverted = isSensorInverted;
        return this;
    }

    /**
     * Chain setter for the motor inversion.
     * This is used to invert the direction of the motor.
     *
     * @param isInverted Whether the motor is inverted
     */
    public MotorConfig inverted(boolean isInverted) {
        this.isInverted = isInverted;
        return this;
    }

    /**
     * Chain setter for the feedback continuity.
     * If true, not continuous, when it reaches the last tick, it will go back to zero. (e.g. 0-359)
     * If false, continuous, when it reaches the last tick, it will go to that plus one. (e.g. -infinity to infinity)
     *
     * @param feedbackNotContinuous Whether the feedback is not continuous
     */
    public MotorConfig withFeedbackNotContinuous(boolean feedbackNotContinuous) {
        this.feedbackNotContinuous = feedbackNotContinuous;
        return this;
    }

    /**
     * Chain setter for the feedback device.
     * This is the sensor that the motor controller will use to report speed and position.
     *
     * @param feedbackDevice The feedback device
     * @param pidIdx         The PID slot.
     */
    public MotorConfig withFeedbackDevice(FeedbackDevice feedbackDevice, int pidIdx) {
        if(pidIdx == 0) {
            primaryFeedbackDevice = feedbackDevice;
        } else {
            secondaryFeedbackDevice = feedbackDevice;
        }
        return this;
    }

    /**
     * Chain setter for the primary feedback device.
     * This is the sensor that the motor controller will use to report speed and position.
     *
     * @param feedbackDevice The feedback device
     */
    public MotorConfig withPrimaryFeedbackDevice(FeedbackDevice feedbackDevice) {
        primaryFeedbackDevice = feedbackDevice;
        return this;
    }

    /**
     * Chain setter for the secondary feedback device.
     * This is the sensor that the motor controller will use to report speed and position.
     *
     * @param feedbackDevice The feedback device
     */
    public MotorConfig withSecondaryFeedbackDevice(FeedbackDevice feedbackDevice) {
        secondaryFeedbackDevice = feedbackDevice;
        return this;
    }

    /**
     * Chain setter for the remote sensor source.
     * This determines the type and id of the sensor that is connected to the remote slot.
     *
     * @param remoteSensorSourceDeviceId The remote sensor source device ID
     * @param remoteSensorSourceType     The remote sensor source type
     * @param slot                       The slot (0 or 1)
     */
    public MotorConfig withRemoteSensorSource(
            int remoteSensorSourceDeviceId, RemoteSensorSource remoteSensorSourceType, int slot) {
        if(slot == 0) {
            remoteSensorSource0DeviceId = remoteSensorSourceDeviceId;
            remoteSensorSource0Type = remoteSensorSourceType;
        } else {
            remoteSensorSource1DeviceId = remoteSensorSourceDeviceId;
            remoteSensorSource1Type = remoteSensorSourceType;
        }
        return this;
    }
}