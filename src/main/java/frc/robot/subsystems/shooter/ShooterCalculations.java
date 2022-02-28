package frc.robot.subsystems.shooter;

import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShooterCalculations {

    /**
     * @return zone the robot is in based on the distance from the hub
     */
    private static int calculateZone(double heightSeen) {
        if(heightSeen >= ShooterConstants.ZONE_LIMITS[0])
            return 1;
        else if(heightSeen >= ShooterConstants.ZONE_LIMITS[1])
            return 2;
        else if(heightSeen >= ShooterConstants.ZONE_LIMITS[3])
            return 3;
        else
            return 4;
    }

    /**
     * Calculates the optimal velocity of the shooter
     * based on the zone the robot is in using a trend-line.
     *
     * @return target velocity of the shooter
     */
    public static double calculateVelocity(double heightSeen) {
        double mCoef = 0;
        double bCoef = 0;

        switch(calculateZone(heightSeen)) {
            case 1:
                mCoef = ShooterConstants.ZONE_1_COEFS[0];
                bCoef = ShooterConstants.ZONE_1_COEFS[1];
                break;
            case 2:
                mCoef = ShooterConstants.ZONE_2_COEFS[0];
                bCoef = ShooterConstants.ZONE_2_COEFS[1];
                break;
            case 3:
                mCoef = ShooterConstants.ZONE_3_COEFS[0];
                bCoef = ShooterConstants.ZONE_3_COEFS[1];
                break;
            default:
                mCoef = ShooterConstants.ZONE_4_COEFS[0];
                bCoef = ShooterConstants.ZONE_4_COEFS[1];
                break;
        }

        return mCoef * heightSeen + bCoef;
    }

    /**
     * Calculates the optimal angle of the pitcher
     * based on the zone the robot is in.
     *
     * @return target angle of the pitcher
     */
    public static double calculateAngle(double heightSeen) {
        switch(calculateZone(heightSeen)) {
            case 1:
                return ShooterConstants.ANGLES[0];
            case 2:
                return ShooterConstants.ANGLES[1];
            case 3:
                return ShooterConstants.ANGLES[2];
            default:
                return ShooterConstants.ANGLES[3];
        }
    }
}
