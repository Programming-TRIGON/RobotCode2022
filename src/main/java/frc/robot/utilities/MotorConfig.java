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
    private FeedbackDevice feedbackDevice;
    private RemoteSensorSource remoteSensorSourceType;
    private int remoteSensorSourceDeviceId;
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
        feedbackDevice = FeedbackDevice.None;
        remoteSensorSourceType = RemoteSensorSource.Off;
        remoteSensorSourceDeviceId = 0;
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
        feedbackDevice = config.getFeedbackDevice();
        remoteSensorSourceType = config.getRemoteSensorSourceType();
        remoteSensorSourceDeviceId = config.getRemoteSensorSourceDeviceId();
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

    public FeedbackDevice getFeedbackDevice() {
        return feedbackDevice;
    }

    public RemoteSensorSource getRemoteSensorSourceType() {
        return remoteSensorSourceType;
    }

    public PIDFCoefs getCoefs() {
        return coefs;
    }

    public int getRemoteSensorSourceDeviceId() {
        return remoteSensorSourceDeviceId;
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
     */
    public MotorConfig withFeedbackDevice(FeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
        return this;
    }

    /**
     * Chain setter for the remote sensor source type.
     * This determines the type of sensor that is connected to the remote slot.
     *
     * @param remoteSensorSourceType     The remote sensor source type
     * @param remoteSensorSourceDeviceId The remote sensor source device ID
     */
    public MotorConfig withRemoteSensorSource(
            int remoteSensorSourceDeviceId,
            RemoteSensorSource remoteSensorSourceType) {
        this.remoteSensorSourceDeviceId = remoteSensorSourceDeviceId;
        this.remoteSensorSourceType = remoteSensorSourceType;
        return this;
    }
}