package frc.robot.constants;

import com.google.gson.annotations.SerializedName;
import frc.robot.utilities.JsonHandler;
import frc.robot.utilities.pid.PIDCoefs;
import frc.robot.utilities.pid.PIDFCoefs;

public class LocalConstants {
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

    public LocalConstants() {
        localSwerveConstants = new LocalSwerveConstants();
        localShooterConstants = new LocalShooterConstants();
        localIntakeOpenerConstants = new LocalIntakeOpenerConstants();
        localPitcherConstants = new LocalPitcherConstants();
        localLoaderConstants = new LocalLoaderConstants();
        localClimberConstants = new LocalClimberConstants();
    }

    public void write() {
        JsonHandler.write(this);
    }

    public static class LocalPitcherConstants {
        PIDFCoefs pidfCoefs;
        public int encoderOffset;

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
        PIDCoefs turnToTargetCoefs;
        PIDCoefs driveToTargetCoefs;
        LocalSwerveModules modules;

        public LocalSwerveConstants() {
            turnToTargetCoefs = new PIDCoefs();
            driveToTargetCoefs = new PIDCoefs();
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
                public double encoderOffset;

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
}
