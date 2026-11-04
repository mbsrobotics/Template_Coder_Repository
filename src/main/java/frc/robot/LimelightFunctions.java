
package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class LimelightFunctions extends SubsystemBase {

    private final NetworkTableInstance nt =
            NetworkTableInstance.getDefault();

    private final Map<String, CameraData> cameraData =
            new HashMap<>();

    private static final double CAMERA_TIMEOUT_SECONDS = 2.0;

    private static class CameraData {
        final NetworkTable table;
        final NetworkTableEntry heartbeat;
        final NetworkTableEntry tv;
        final NetworkTableEntry tid;
        final NetworkTableEntry tx;
        final NetworkTableEntry ty;
        final NetworkTableEntry pose;
        final NetworkTableEntry rawFiducials;

        double lastHeartbeat = Double.NaN;
        double lastHeartbeatTime;

        boolean hasTarget;
        int tagID = -1;
        double horizontalOffset;
        double verticalOffset;
        double forwardDistance;

        final List<AprilTagData> visibleTags = new ArrayList<>();

        CameraData(NetworkTable table) {
            this.table = table;
            heartbeat = table.getEntry("hb");
            tv = table.getEntry("tv");
            tid = table.getEntry("tid");
            tx = table.getEntry("tx");
            ty = table.getEntry("ty");
            pose = table.getEntry("targetpose_cameraspace");
            rawFiducials = table.getEntry("rawfiducials");
            lastHeartbeatTime = Timer.getFPGATimestamp();
        }
    }

    public static class AprilTagData {
        public final int id;
        public final double tx;
        public final double ty;
        public final double area;
        public final double distanceToCamera;
        public final double distanceToRobot;
        public final double ambiguity;

        public AprilTagData(
                int id,
                double tx,
                double ty,
                double area,
                double distanceToCamera,
                double distanceToRobot,
                double ambiguity) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.area = area;
            this.distanceToCamera = distanceToCamera;
            this.distanceToRobot = distanceToRobot;
            this.ambiguity = ambiguity;
        }
    }

    public LimelightFunctions() {
        detectCameras();
    }

    private void detectCameras() {
        double now = Timer.getFPGATimestamp();

        for (Topic topic : nt.getTopics("/limelight")) {
            String topicName = topic.getName();

            if (!topicName.endsWith("/hb")) {
                continue;
            }

            String cameraName = topicName.substring(
                    1, topicName.length() - 3);

            if (!cameraName.startsWith("limelight")
                    || cameraName.contains("/")) {
                continue;
            }

            NetworkTable table = nt.getTable(cameraName);

            CameraData data = cameraData.computeIfAbsent(
                    cameraName, key -> new CameraData(table));

            double heartbeatValue =
                    data.heartbeat.getDouble(-1.0);

            if (Double.isNaN(data.lastHeartbeat)
                    || heartbeatValue != data.lastHeartbeat) {
                data.lastHeartbeat = heartbeatValue;
                data.lastHeartbeatTime = now;
            }
        }

        cameraData.entrySet().removeIf(entry ->
                now - entry.getValue().lastHeartbeatTime
                        > CAMERA_TIMEOUT_SECONDS);
    }

    @Override
    public void periodic() {
        detectCameras();

        for (CameraData data : cameraData.values()) {
            data.hasTarget = data.tv.getDouble(0.0) == 1.0;
            data.visibleTags.clear();

            if (data.hasTarget) {
                data.tagID = (int) data.tid.getDouble(-1.0);
                data.horizontalOffset = data.tx.getDouble(0.0);
                data.verticalOffset = data.ty.getDouble(0.0);

                double[] pose =
                        data.pose.getDoubleArray(new double[0]);

                data.forwardDistance = pose.length >= 3
                        ? Math.abs(pose[2])
                        : 0.0;
            } else {
                data.tagID = -1;
                data.horizontalOffset = 0.0;
                data.verticalOffset = 0.0;
                data.forwardDistance = 0.0;
            }

            updateVisibleTags(data);
        }

        updateDashboard();
    }

    private void updateVisibleTags(CameraData data) {
        double[] raw =
                data.rawFiducials.getDoubleArray(new double[0]);

        for (int i = 0; i + 7 <= raw.length; i += 7) {
            data.visibleTags.add(new AprilTagData(
                    (int) raw[i],
                    raw[i + 1],
                    raw[i + 2],
                    raw[i + 3],
                    raw[i + 4],
                    raw[i + 5],
                    raw[i + 6]));
        }
    }

    private CameraData getData(String cameraName) {
        if (cameraName == null) {
            return null;
        }

        String name = cameraName;
        while (name.startsWith("/")) {
            name = name.substring(1);
        }

        return cameraData.get(name);
    }

    public NetworkTable getCamera(String cameraName) {
        CameraData data = getData(cameraName);
        return data != null ? data.table : null;
    }

    public List<String> getCameraNames() {
        return new ArrayList<>(cameraData.keySet());
    }

    public int getCameraCount() {
        return cameraData.size();
    }

    public boolean hasCamera(String cameraName) {
        return getData(cameraName) != null;
    }

    public boolean cameraHasTarget(String cameraName) {
        CameraData data = getData(cameraName);
        return data != null && data.hasTarget;
    }

    public int getCameraTagID(String cameraName) {
        CameraData data = getData(cameraName);
        return data != null ? data.tagID : -1;
    }

    public double getCameraTx(String cameraName) {
        CameraData data = getData(cameraName);
        return data != null ? data.horizontalOffset : 0.0;
    }

    public double getCameraTy(String cameraName) {
        CameraData data = getData(cameraName);
        return data != null ? data.verticalOffset : 0.0;
    }

    public double getCameraForwardDistanceMeters(String cameraName) {
        CameraData data = getData(cameraName);
        return data != null ? data.forwardDistance : 0.0;
    }

    public double getCameraForwardDistanceFeet(String cameraName) {
        return getCameraForwardDistanceMeters(cameraName) * 3.28084;
    }

    public int getCameraVisibleTagCount(String cameraName) {
        CameraData data = getData(cameraName);
        return data != null ? data.visibleTags.size() : 0;
    }

    public List<AprilTagData> getVisibleTags(String cameraName) {
        CameraData data = getData(cameraName);

        return data != null
                ? new ArrayList<>(data.visibleTags)
                : new ArrayList<>();
    }

    public int[] getVisibleTagIDs(String cameraName) {
        CameraData data = getData(cameraName);

        if (data == null) {
            return new int[0];
        }

        int[] ids = new int[data.visibleTags.size()];

        for (int i = 0; i < ids.length; i++) {
            ids[i] = data.visibleTags.get(i).id;
        }

        return ids;
    }

    public boolean canCameraSeeTag(String cameraName, int desiredID) {
        return getCameraTagData(cameraName, desiredID) != null;
    }

    public AprilTagData getCameraTagData(
            String cameraName, int desiredID) {
        CameraData data = getData(cameraName);

        if (data == null) {
            return null;
        }

        for (AprilTagData tag : data.visibleTags) {
            if (tag.id == desiredID) {
                return tag;
            }
        }

        return null;
    }

    public double getCameraTagTx(String cameraName, int desiredID) {
        AprilTagData tag = getCameraTagData(cameraName, desiredID);
        return tag != null ? tag.tx : 0.0;
    }

    public double getCameraTagTy(String cameraName, int desiredID) {
        AprilTagData tag = getCameraTagData(cameraName, desiredID);
        return tag != null ? tag.ty : 0.0;
    }

    public double getCameraTagDistanceMeters(
            String cameraName, int desiredID) {
        AprilTagData tag = getCameraTagData(cameraName, desiredID);
        return tag != null ? tag.distanceToCamera : 0.0;
    }

    public double getCameraTagDistanceFeet(
            String cameraName, int desiredID) {
        return getCameraTagDistanceMeters(cameraName, desiredID)
                * 3.28084;
    }

    public void printVisibleTags(String cameraName) {
        CameraData data = getData(cameraName);

        if (data == null) {
            System.out.println(cameraName + " is not detected.");
            return;
        }

        if (data.visibleTags.isEmpty()) {
            System.out.println(cameraName + " sees no AprilTags.");
            return;
        }

        System.out.print(cameraName + " sees tags: ");

        for (AprilTagData tag : data.visibleTags) {
            System.out.print(tag.id + " ");
        }

        System.out.println();
    }

    private void updateDashboard() {
        NetworkTable dashboard =
                nt.getTable("SmartDashboard").getSubTable("Limelight");

        dashboard.getEntry("Detected Camera Count")
                .setDouble(cameraData.size());

        NetworkTable camerasTable =
                dashboard.getSubTable("Detected Cameras");

        for (Map.Entry<String, CameraData> entry
                : cameraData.entrySet()) {
            String cameraName = entry.getKey();
            CameraData data = entry.getValue();

            NetworkTable cameraTable =
                    camerasTable.getSubTable(cameraName);

            cameraTable.getEntry("Connected").setBoolean(true);
            cameraTable.getEntry("Has Target")
                    .setBoolean(data.hasTarget);
            cameraTable.getEntry("Tag ID").setDouble(data.tagID);
            cameraTable.getEntry("TX").setDouble(data.horizontalOffset);
            cameraTable.getEntry("TY").setDouble(data.verticalOffset);

            cameraTable.getEntry("Forward Distance (m)")
                    .setDouble(data.forwardDistance);

            cameraTable.getEntry("Forward Distance (ft)")
                    .setDouble(data.forwardDistance * 3.28084);

            cameraTable.getEntry("Visible Tag Count")
                    .setDouble(data.visibleTags.size());

            NetworkTable tagsTable = cameraTable.getSubTable("Tags");

            for (AprilTagData tag : data.visibleTags) {
                NetworkTable tagTable =
                        tagsTable.getSubTable("Tag " + tag.id);

                tagTable.getEntry("ID").setDouble(tag.id);
                tagTable.getEntry("TX").setDouble(tag.tx);
                tagTable.getEntry("TY").setDouble(tag.ty);
                tagTable.getEntry("Area").setDouble(tag.area);

                tagTable.getEntry("Distance To Camera (m)")
                        .setDouble(tag.distanceToCamera);

                tagTable.getEntry("Distance To Robot (m)")
                        .setDouble(tag.distanceToRobot);

                tagTable.getEntry("Ambiguity")
                        .setDouble(tag.ambiguity);
            }
        }
    }
}
