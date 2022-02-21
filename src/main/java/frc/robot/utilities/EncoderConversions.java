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
     * @param velocityCounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocityCounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocityCounts, gearRatio);
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
     * @param magTicks the ticks from the mag sensor
     * @return the angle in degrees
     */
    public static double MagToDegrees(double magTicks, double gearRatio) {
        return (magTicks) / 4096f * 360 / gearRatio;
    }

    public static double MagToDegrees(double magTick) {
        return MagToDegrees(magTick, 1);
    }

    /**
     * @param degrees angle of the wheel or gear in degrees
     * @param radius  radius of the wheel or pitch radius of the gear
     * @return amount of distance the system has traveled in meters
     */
    public static double degreesToMeters(double degrees, double radius) {
        double circumference = 2 * Math.PI * radius;
        return degrees * circumference / 360;
    }

    /**
     * @param distance amount of distance the system has traveled in meters
     * @param radius   radius of the wheel or pitch radius of the gear
     * @return the angle of the wheel or gear in degrees
     */
    public static double metersToDegrees(double distance, double radius) {
        double circumference = 2 * Math.PI * radius;
        return distance / circumference * 360;
    }
}