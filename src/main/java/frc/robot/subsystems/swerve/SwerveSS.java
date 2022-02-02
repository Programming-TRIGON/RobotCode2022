package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.Pigeon;
import frc.robot.constants.RobotConstants.SwerveConstants;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Modules;

/**
 * The Swerve subsystem.
 */
public class SwerveSS extends SubsystemBase implements TestableSubsystem {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveModules;
    private final Pigeon gyro;

    public SwerveSS() {
        gyro = SwerveConstants.PIGEON;
        gyro.configFactoryDefault();
        resetGyro();

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getAngle());

        swerveModules = new SwerveModule[4];
        swerveModules[Modules.FRONT_LEFT.getId()] = new SwerveModule(SwerveConstants.FRONT_LEFT_CONSTANTS);
        swerveModules[Modules.FRONT_RIGHT.getId()] = new SwerveModule(SwerveConstants.FRONT_RIGHT_CONSTANTS);
        swerveModules[Modules.REAR_LEFT.getId()] = new SwerveModule(SwerveConstants.REAR_LEFT_CONSTANTS);
        swerveModules[Modules.REAR_RIGHT.getId()] = new SwerveModule(SwerveConstants.REAR_RIGHT_CONSTANTS);
    }

    /**
     * Drives the swerve by the given values
     *
     * @param translation   The translation to apply, where x is forward and y is to the left
     * @param rotation      The rotation to apply, in radians per second
     * @param fieldRelative Whether the translation and rotation are field-relative
     * @param isOpenLoop    Whether we should drive the modules in open loop, or in closed loop, with a PID loop for
     *                      each module's speed
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // set the desired states based on the given
        // translation and rotation
        SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds( // if field relative, convert to field relative speeds
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getAngle()
                ) :
                new ChassisSpeeds( // if not field relative, just use the given translation and rotation
                        translation.getX(),
                        translation.getY(),
                        rotation
                )
        );
        // making sure the speeds are within the max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        // if we are not moving or rotating at all, set the desired angle to the current angle
        if(translation.getNorm() == 0 && rotation == 0) {
            for(int i = 0; i < swerveModules.length; i++) {
                swerveModuleStates[i].angle = swerveModules[i].getAngle();
            }
        }

        // set the desired state for each module
        for(int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    /**
     * Sets each module by the given array
     *
     * @param desiredStates array of length 4, where each element is the desired state of the module
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

        for(int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i], true);
        }
    }

    /**
     * Returns the current pose of the robot as reported by the odometry
     *
     * @return the current pose of the robot as reported by the odometry
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the given pose
     *
     * @param pose the pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getAngle());
    }

    /**
     * Returns the current state of each module
     *
     * @return the current state of each module
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * Calibrates and zeroes the gyro
     */
    public void resetGyro() {
        gyro.calibrate();
    }

    /**
     * Returns the current yaw (angle) of the robot
     *
     * @return the current yaw of the robot
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    @Override
    public void move(double power) {
        drive(new Translation2d(0, 0), power, false, true);
    }

    @Override
    public void periodic() {
        // update the odometry with the angle and the current state of each module
        swerveOdometry.update(getAngle(), getStates());
    }

    @Override
    public double[] getValues() {
        double[] values = new double[12];
        for(int i = 0; i < swerveModules.length; i++) {
            double[] moduleValues = swerveModules[i].getEncoderVelocities();
            System.arraycopy(moduleValues, 0, values, i * 3, moduleValues.length);
        }
        return values;
    }
}