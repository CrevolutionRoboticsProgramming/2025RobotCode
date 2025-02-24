package frc.robot.algaepivot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeShooterInterpolation extends SubsystemBase{
    // Create an InterpolatingTreeMap to store data points with double keys
    private static InterpolatingTreeMap<Double, Double> dataAngle = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static AlgaeShooterInterpolation mInstance;
    private double algaeShooterPivotAngle;
    private double distance;
    // private static InterpolatingDoubleTreeMap data = new InterpolatingDoubleTreeMap();
    public AlgaeShooterInterpolation() {
        // Configures the TreeMap
        configureInterpolatingAngleTreeMap();
    }

    public static AlgaeShooterInterpolation getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeShooterInterpolation();
        }
        return mInstance;
    }

    // Method to set up data values for Angles in the TreeMap
    private void configureInterpolatingAngleTreeMap() {
        // Add data points to the TreeMap
        dataAngle.put(0.0, 0.0);
        dataAngle.put(1.5, 0.0);
        dataAngle.put(3.89, 29.0);
        dataAngle.put(5.3, 26.5);
        
    }

    // Method to interpolate the data point at a specific distance
    public double getInterpolatedAngle(double distance) {
        this.distance = distance;

        /* Get the interpolated entry from the TreeMap for the specified distance */
        algaeShooterPivotAngle = dataAngle.get(distance);

        // Return the interpolated data point
        return algaeShooterPivotAngle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Interpolated Angle]", algaeShooterPivotAngle);
        SmartDashboard.putNumber("[Interpolated Passed Dist.]", distance);
    }
}
