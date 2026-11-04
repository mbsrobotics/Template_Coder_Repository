// File: LimelightThermal.java
package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightThermal {

    private final NetworkTable limelightTable;

    public LimelightThermal(String limelightName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    public void setThrottle(int value) {
        limelightTable.getEntry("throttle").setNumber(value);
    }

    public void onDisabled() {
        setThrottle(200); // Example: 100-200 while disabled
    }

    public void onEnabled() {
        setThrottle(0); // Normal operation when enabled
    }
}
