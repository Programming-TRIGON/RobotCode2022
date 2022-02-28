package frc.robot.constants;

import com.google.gson.annotations.SerializedName;
import frc.robot.utilities.JsonHandler;
import frc.robot.utilities.pid.PIDCoefs;
import frc.robot.utilities.pid.PIDFCoefs;
import frc.robot.utilities.pid.TrigonPIDController;

public class LocalConstants {
    @SerializedName("Driver")
    public LocalDriverConstants localDriverConstants;
    @SerializedName("Swerve")
    public LocalSwerveConstants localSwerveConstants;
    @SerializedName("Shooter")
    public LocalShooterConstants localShooterConstants;
    @SerializedName("IntakeOpener")
    public LocalIntakeOpenerConstants localIntakeOpenerConstants;
    @SerializedName("Pitcher")
    public LocalPitcherConstants localPitcherConstants;
    @SerializedName("Loader")
    public LocalLoaderConstants localLoaderConstants;
    @SerializedName("Climber")
    public LocalClimberConstants localClimberConstants;
    @SerializedName("MotionProfiling")
    public LocalMotionProfilingConstants localMotionProfilingConstants;

    public LocalConstants() {
        localDriverConstants = new LocalDriverConstants();
        localSwerveConstants = new LocalSwerveConstants();
        localShooterConstants = new LocalShooterConstants();
        localIntakeOpenerConstants = new LocalIntakeOpenerConstants();
        localPitcherConstants = new LocalPitcherConstants();
        localLoaderConstants = new LocalLoaderConstants();
        localClimberConstants = new LocalClimberConstants();
        localMotionProfilingConstants = new LocalMotionProfilingConstants();
    }

    public void write() {
        JsonHandler.write(this);
    }

    public static class LocalDriverConstants {
        int drivingSpeedDivider;
    }

    public static class LocalPitcherConstants {
        PIDFCoefs pidfCoefs;

        public LocalPitcherConstants() {
            pidfCoefs = new PIDFCoefs();
        }
    }

    public static class LocalIntakeOpenerConstants {
        PIDFCoefs pidfCoefs;

        public LocalIntakeOpenerConstants() {
            pidfCoefs = new PIDFCoefs();
        }
    }

    public static class LocalLoaderConstants {
        PIDFCoefs pidfCoefs;

        public LocalLoaderConstants() {
            pidfCoefs = new PIDFCoefs();
        }
    }

    public static class LocalClimberConstants {
        PIDFCoefs pidfCoefs;

        public LocalClimberConstants() {
            pidfCoefs = new PIDFCoefs();
        }
    }

    public static class LocalSwerveConstants {
        LocalSwerveModules modules;

        public LocalSwerveConstants() {
            modules = new LocalSwerveModules();
            modules.frontRightModuleConstants = new LocalSwerveModules.LocalSwerveModuleConstants();
            modules.frontLeftModuleConstants = new LocalSwerveModules.LocalSwerveModuleConstants();
            modules.rearRightModuleConstants = new LocalSwerveModules.LocalSwerveModuleConstants();
            modules.rearLeftModuleConstants = new LocalSwerveModules.LocalSwerveModuleConstants();
        }

        public static class LocalSwerveModules {
            @SerializedName("frontRight")
            public LocalSwerveModuleConstants frontRightModuleConstants;
            @SerializedName("frontLeft")
            public LocalSwerveModuleConstants frontLeftModuleConstants;
            @SerializedName("rearRight")
            public LocalSwerveModuleConstants rearRightModuleConstants;
            @SerializedName("rearLeft")
            public LocalSwerveModuleConstants rearLeftModuleConstants;

            public static class LocalSwerveModuleConstants {
                double encoderOffset;

                PIDFCoefs angleCoefs;
                PIDFCoefs driveCoefs;

                public LocalSwerveModuleConstants() {
                    encoderOffset = 0;
                    angleCoefs = new PIDFCoefs();
                    driveCoefs = new PIDFCoefs();
                }
            }
        }
    }

    public static class LocalShooterConstants {
        PIDFCoefs pidfCoefs;

        public LocalShooterConstants() {
            pidfCoefs = new PIDFCoefs();
        }
    }

    public static class LocalMotionProfilingConstants {
        PIDCoefs X_PID_CONTROLLER;
        PIDCoefs Y_PID_CONTROLLER;
        PIDCoefs THETA_PID_CONTROLLER;

        public LocalMotionProfilingConstants() {
            X_PID_CONTROLLER = new PIDCoefs();
            Y_PID_CONTROLLER = new PIDCoefs();
            THETA_PID_CONTROLLER = new PIDCoefs();
        }
    }
}
