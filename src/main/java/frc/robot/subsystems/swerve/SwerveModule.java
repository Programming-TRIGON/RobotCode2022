package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.SwerveConstants;
import frc.robot.constants.SwerveModuleConstants;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.Savable;
import frc.robot.utilities.pid.PIDFTalonFX;

/**
 * This class represents a single swerve module.
 * The module has two motors, one for the drive and one for the angle.
 * There's a MAG encoder for the angle.
 */
public class SwerveModule implements Savable {
    private final PIDFTalonFX angleMotor;
    private final PIDFTalonFX driveMotor;
    private final TrigonTalonSRX angleEncoder;
    private final SwerveModuleConstants constants;
    private double encoderOffset;
    private SwerveModuleState lastDesiredState;
    private boolean isTuning;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        this.constants = moduleConstants;

        angleEncoder = moduleConstants.angleEncoder;
        angleMotor = moduleConstants.angleMotor;
        driveMotor = moduleConstants.driveMotor;

        encoderOffset = constants.localConstants.encoderOffset;

        resetToAbsolute();
        driveMotor.ce_setSelectedSensorPosition(0);
        lastDesiredState = getState();
        putOnShuffleboard(constants.module.getName());
    }

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if(Math.abs(delta) > 90) {
            targetSpeed *= -1;
            targetAngle -= delta > 90 ? 180 : -180;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(
            double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound) {
            newAngle += 360;
        }
        while(newAngle > upperBound) {
            newAngle -= 360;
        }
        if(newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if(newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public double getEncoderOffset() {
        return encoderOffset;
    }

    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
        resetToAbsolute();
    }

    public double[] getEncoderVelocities() {
        return new double[] {
                angleEncoder.getSelectedSensorVelocity(),
                angleMotor.getSelectedSensorVelocity(),
                driveMotor.getSelectedSensorVelocity()
        };
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isTuning) {
        // If we are called by a command or something alike, that has nothing to do with tuning, then we ignore the call
        if(!isTuning && isTuning())
            return;
        // Custom optimize command, since default WPILib optimize assumes
        // continuous controller which CTRE is not
        desiredState = optimize(
                desiredState,
                getState().angle
        );
        if(isOpenLoop) {
            /* If we're in open loop, we don't want to use the PID controller,
            and we just set the drive speed in percentage of the max speed */
            double percentOutput =
                    desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            /* If we're in closed loop, we want to use the PID controller,
            and we set the drive speed in meters per second  */
            double velocity = Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond,
                    SwerveConstants.WHEEL_CIRCUMFERENCE,
                    SwerveConstants.DRIVE_GEAR_RATIO);
            // Sets the drive motor velocity with the driveFeedforward.
            driveMotor.setSetpoint(velocity);
        }

        double desiredAngle = desiredState.angle.getDegrees();
        // Set the angle motor's position to the desired angle.
        angleMotor.setSetpoint(
                Conversions.degreesToFalcon(
                        desiredAngle,
                        SwerveConstants.ANGLE_GEAR_RATIO));
        lastDesiredState = desiredState;
    }

    /**
     * Sets the value of the integrated angle encoder to the absolute value of the
     * angle,
     * from the absolute encoder, and the absolute encoder offset.
     */
    private void resetToAbsolute() {
        // calculate absolute position and convert to Falcon units
        double absolutePosition = Conversions.degreesToFalcon(
                getAngle().getDegrees() - encoderOffset,
                SwerveConstants.ANGLE_GEAR_RATIO);
        // Set the integrated angle encoder to the absolute position.
        angleMotor.ce_setSelectedSensorPosition((int) absolutePosition, 0);
    }

    /**
     * Returns the angle of the module in degrees, using the angle encoder.
     *
     * @return the angle of the module in degrees
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.magToDegrees(angleEncoder.getSelectedSensorPosition(), 1));
    }

    /**
     * Returns the current state of the module.
     * The state is composed of the angle that we get from the angle encoder, and
     * the speed of the drive motor.
     *
     * @return the state of the module
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(
                driveMotor.getSelectedSensorVelocity(),
                SwerveConstants.WHEEL_CIRCUMFERENCE,
                SwerveConstants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(
                Conversions.falconToDegrees(
                        angleMotor.getSelectedSensorPosition(),
                        SwerveConstants.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public void setDesiredAngle(double angle, boolean isOpenLoop, boolean isTuning) {
        setDesiredState(
                new SwerveModuleState(
                        getLastDesiredState().speedMetersPerSecond,
                        Rotation2d.fromDegrees(angle)),
                isOpenLoop,
                isTuning);
    }

    public void setDesiredSpeed(double speed, boolean isOpenLoop, boolean isTuning) {
        setDesiredState(
                new SwerveModuleState(
                        speed,
                        getLastDesiredState().angle),
                isOpenLoop,
                isTuning);
    }

    /**
     * Returns the desired state of the module.
     *
     * @return the desired state of the module
     */
    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }

    /**
     * sets the feedforward values of the drive motor
     *
     * @param kV drive motor velocity gain
     * @param kS drive motor static gain
     */
    public void setDriveFeedforward(double kV, double kS) {
        driveMotor.setKV(kV);
        driveMotor.setKS(kS);
    }

    public boolean isTuning() {
        return isTuning;
    }

    public void setTuning(boolean tuning) {
        isTuning = tuning;
    }

    public void putOnShuffleboard(String name) {
        SmartDashboard.putData("Swerve/" + name + "/Stats", this);
        SmartDashboard.putData("Swerve/" + name + "/Angle Motor", angleMotor);
        SmartDashboard.putData("Swerve/" + name + "/Drive Motor", driveMotor);
    }

    @Override
    public void load() {
        setEncoderOffset(constants.localConstants.encoderOffset);
    }

    @Override
    public void save() {
        constants.localConstants.encoderOffset = getEncoderOffset();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "Desired Angle",
                () -> getLastDesiredState().angle.getDegrees(),
                x -> {
                    if(isTuning())
                        setDesiredAngle(x, false, true);
                });
        builder.addDoubleProperty(
                "Angle",
                () -> getState().angle.getDegrees(),
                null);
        builder.addDoubleProperty(
                "Desired Speed",
                () -> getLastDesiredState().speedMetersPerSecond
                , speed -> {
                    if(isTuning())
                        setDesiredSpeed(speed, false, isTuning);
                });
        builder.addDoubleProperty(
                "Speed",
                () -> getState().speedMetersPerSecond
                , null);
        builder.addDoubleProperty(
                "Angle Error",
                () -> Conversions.falconToDegrees(
                        angleMotor.getClosedLoopError(), SwerveConstants.ANGLE_GEAR_RATIO)
                , null);
        builder.addDoubleProperty(
                "Encoder Offset",
                this::getEncoderOffset,
                (offset) -> {
                    if(isTuning())
                        setEncoderOffset(offset);
                });
        builder.addDoubleProperty(
                "Raw Angle",
                () -> this.getAngle().getDegrees(),
                null);
        builder.addBooleanProperty(
                "Is Tuning",
                this::isTuning
                , this::setTuning);
    }
}