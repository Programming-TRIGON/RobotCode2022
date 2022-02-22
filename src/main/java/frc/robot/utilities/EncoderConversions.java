package frc.robot.utilities;

public class EncoderConversions {

    /**
     * @param counts    Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / 360 * (gearRatio * 2048.0);
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * Default gear ratio of 1
     *
     * @param velocityCounts Falcon Velocity Counts
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts) {
        return falconToRPM(velocityCounts, 1);
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * Default gear ratio of 1
     *
     * @param RPM RPM of mechanism
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM) {
        return RPMToFalcon(RPM, 1);
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param magTicks  angle of the Mag sensor in ticks
     * @param gearRatio Gear Ratio between Mag and Mechanism (set to 1 for Mag RPM)
     * @return the angle in degrees
     */
    public static double magToDegrees(double magTicks, double gearRatio) {
        return magTicks / 4096f * 360 / gearRatio;
    }

    public static double magToDegrees(double magTick) {
        return magToDegrees(magTick, 1);
    }

    /**
     * @param degrees   angle of the wheel or gear in degrees
     * @param gearRatio Gear Ratio between Mag and Mechanism (set to 1 for Mag RPM)
     * @return the angle of the Mag in ticks
     */
    public static double degreesToMag(double degrees, double gearRatio) {
        return degrees * 4096f / 360 * gearRatio;
    }

    public static double degreesToMag(double degrees) {
        return degreesToMag(degrees, 1);
    }
}