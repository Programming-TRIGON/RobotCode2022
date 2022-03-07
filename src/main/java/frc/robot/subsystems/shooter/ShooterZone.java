package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class ShooterZone {
    private double pitcherAngle;
    private ArrayList<ShooterWaypoint> waypoints;

    public ShooterZone(double pitcherAngle, ShooterWaypoint... waypoints) {
        this.pitcherAngle = pitcherAngle;
        this.waypoints = new ArrayList<>(Arrays.asList(waypoints));
        Collections.sort(this.waypoints);
    }

    public double getPitcherAngle() {
        return pitcherAngle;
    }

    public ArrayList<ShooterWaypoint> getWaypoints() {
        return waypoints;
    }
}
