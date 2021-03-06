package frc.robot.constants;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // TODO: Set variables for hardware components

    protected static class CAN {
        public static class Swerve {
            public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0;
            public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 1;
            public static final int FRONT_LEFT_ANGLE_ENCODER_ID = 2;

            public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
            public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
            public static final int FRONT_RIGHT_ANGLE_ENCODER_ID = 5;

            public static final int REAR_LEFT_DRIVE_MOTOR_ID = 6;
            public static final int REAR_LEFT_ANGLE_MOTOR_ID = 7;
            public static final int REAR_LEFT_ANGLE_ENCODER_ID = 8;

            public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 9;
            public static final int REAR_RIGHT_ANGLE_MOTOR_ID = 10;
            public static final int REAR_RIGHT_ANGLE_ENCODER_ID = 11;

            public static final int PIGEON_ID = 12;
        }

        public static class Shooter {
            public static final int LEFT_MOTOR_ID = 13;
            public static final int RIGHT_MOTOR_ID = 14;
        }

        public static class Pitcher {
            public static final int MOTOR_ID = 16;
        }

        public static class Loader {
            public static final int MOTOR_ID = 17;
        }

        public static class Climber {
            public static final int LEFT_MOTOR_ID = 18;
            public static final int RIGHT_MOTOR_ID = 19;
        }
    }

    protected static class PCM {

    }

    protected static class DIO {

    }

    protected static class PWM {
        public static class LED {
            public static final int CONTROLLER_PORT = 9;
        }
    }
}
