package frc.robot.motion_profiling;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.RobotConstants.MotionProfilingConstants;
import frc.robot.constants.RobotConstants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSS;

public class TrigonSwerveControllerCG extends SequentialCommandGroup {
    /**
     * A command that uses two PID controllers ({@link PIDController}) and a
     * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
     * {@link Trajectory} with a swerve drive.
     *
     * <p>
     * This command outputs the raw desired Swerve Module States
     * ({@link SwerveModuleState}) in an array. The desired wheel and module
     * rotation velocities should be taken from those and used in velocity PIDs.
     *
     * <p>
     * The robot angle controller does not follow the angle given by the trajectory
     * but rather goes to the angle given in the final state of the trajectory.
     */
    public TrigonSwerveControllerCG(SwerveSS swerveSS, AutoPath path) {
        //constants.THETA_PID_CONTROLLER.enableContinuousInput(-180, 180);
        MotionProfilingConstants.THETA_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putData("TrigonSwerveControllerCMDGP/PID X", MotionProfilingConstants.X_PID_CONTROLLER);
        SmartDashboard.putData("TrigonSwerveControllerCMDGP/PID Y", MotionProfilingConstants.Y_PID_CONTROLLER);
        SmartDashboard.putData(
                "TrigonSwerveControllerCMDGP/PID Rotation", MotionProfilingConstants.THETA_PID_CONTROLLER);

        addCommands(
                new InstantCommand(() ->
                {
                    swerveSS.stopMoving();
                    swerveSS.SetDriveMotorRampRates(0);
                    swerveSS.resetOdometry(
                            path.getPath(swerveSS).getTrajectory().getInitialPose()
                    );
                }, swerveSS),
                new SwerveControllerCommand(
                        path.getPath(swerveSS).getTrajectory(),
                        swerveSS::getPose,
                        SwerveConstants.SWERVE_KINEMATICS,
                        MotionProfilingConstants.X_PID_CONTROLLER,
                        MotionProfilingConstants.Y_PID_CONTROLLER,
                        MotionProfilingConstants.THETA_PID_CONTROLLER,
                        swerveSS::setModuleStates,
                        swerveSS
                ),
                new InstantCommand(() -> {
                    swerveSS.SetDriveMotorRampRates(SwerveConstants.DRIVE_MOTOR_RAMP_RATE);
                    swerveSS.stopMoving();
                }, swerveSS)
        );
    }
}
