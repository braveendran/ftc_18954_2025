package org.firstinspires.ftc.teamcode;

import java.util.TreeMap;

public class DistVelocityProjection {
    private final TreeMap<Double, Double> velocityMap;

    public DistVelocityProjection() {
        velocityMap = new TreeMap<>();
        // These are just placeholders and should be tuned
        velocityMap.put(10.0, 1000.0); // at 10 units of distance, velocity is 1000 RPM
        velocityMap.put(20.0, 1500.0); // at 20 units of distance, velocity is 1500 RPM
        velocityMap.put(30.0, 2000.0); // at 30 units of distance, velocity is 2000 RPM
    }

    /**
     * This function interpolates the velocity for a given distance.
     *
     * @param distance The current distance to the target.
     * @return The interpolated velocity in RPM.
     */
    public double getVelocity(double distance) {
        Double floorKey = velocityMap.floorKey(distance);
        Double ceilKey = velocityMap.ceilingKey(distance);

        if (floorKey == null && ceilKey == null) {
            return 0;
        }
        if (floorKey == null) {
            return velocityMap.get(ceilKey);
        }
        if (ceilKey == null) {
            return velocityMap.get(floorKey);
        }
        if (floorKey.equals(ceilKey)) {
            return velocityMap.get(floorKey);
        }

        double floorValue = velocityMap.get(floorKey);
        double ceilValue = velocityMap.get(ceilKey);

        return floorValue + ((distance - floorKey) * (ceilValue - floorValue) / (ceilKey - floorKey));
    }
}
