
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightAutos {

    private final LimelightFunctions limelight;
    private final DriveSubsystem drive;

    private static final double TARGET_DISTANCE_METERS = 1.0;
    private static final double DISTANCE_TOLERANCE = 0.2;
    private static final double TX_TOLERANCE = 1.5;

    private static final double DISTANCE_KP = 0.8;
    private static final double TURN_KP = 0.035;

    private static final double MAX_FORWARD_SPEED = 0.5;
    private static final double MAX_TURN_SPEED = 0.8;
    private static final double TX_Speed_multiplier = 5.0;

    public LimelightAutos(
            LimelightFunctions limelight,
            DriveSubsystem drive) {
        this.limelight = limelight;
        this.drive = drive;
    }

    public Command followTag(String cameraName) {

        Command command = new Command() {

            @Override
            public void initialize() {
                SmartDashboard.putBoolean(
                        "FollowTag/Running", true);
                stopRobot();
            }

            @Override
            public void execute() {

                boolean connected =
                        limelight.hasCamera(cameraName);

                boolean seesTarget =
                        limelight.cameraHasTarget(cameraName);

                SmartDashboard.putBoolean(
                        "FollowTag/Camera Detected", connected);
                SmartDashboard.putBoolean(
                        "FollowTag/Has Target", seesTarget);

                if (!connected || !seesTarget) {
                    SmartDashboard.putString(
                            "FollowTag/Status",
                            "Waiting for camera or target");
                    stopRobot();
                    return;
                }

                int tagID =
                        limelight.getCameraTagID(cameraName);

                double tx =
                        limelight.getCameraTx(cameraName);

                double distance =
                        limelight.getCameraForwardDistanceMeters(
                                cameraName);

                SmartDashboard.putNumber(
                        "FollowTag/Tag ID", tagID);
                SmartDashboard.putNumber(
                        "FollowTag/TX", tx);
                SmartDashboard.putNumber(
                        "FollowTag/Distance (m)", distance);

                if (!Double.isFinite(distance)
                        || distance <= 0.0) {
                    SmartDashboard.putString(
                            "FollowTag/Status",
                            "Invalid distance");
                    stopRobot();
                    return;
                }

                double error =
                        distance - TARGET_DISTANCE_METERS;

                double forward = 0.0;

                if (Math.abs(error) > DISTANCE_TOLERANCE) {
                    forward = MathUtil.clamp(
                            error * DISTANCE_KP,
                            -MAX_FORWARD_SPEED,
                            MAX_FORWARD_SPEED);
                }

                double rotation = 0.0;

                if (Math.abs(tx) > TX_TOLERANCE) {
                    rotation = MathUtil.clamp(
                            -tx * TURN_KP,
                            -MAX_TURN_SPEED,
                            MAX_TURN_SPEED);
                }

                SmartDashboard.putNumber(
                        "FollowTag/Forward Command", forward);
                SmartDashboard.putNumber(
                        "FollowTag/Rotation Command", rotation);

                SmartDashboard.putString(
                        "FollowTag/Status",
                        Math.abs(error) <= DISTANCE_TOLERANCE
                                && Math.abs(tx) <= TX_TOLERANCE
                                ? "Target reached"
                                : "Following tag");

                drive.driveRobotRelative(
                        new ChassisSpeeds(forward, 0.0, rotation));
            }

            @Override
            public void end(boolean interrupted) {
                stopRobot();
                SmartDashboard.putBoolean(
                        "FollowTag/Running", false);
                SmartDashboard.putString(
                        "FollowTag/Status", "Command ended");
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            private void stopRobot() {
                drive.driveRobotRelative(new ChassisSpeeds());
            }
        };

        command.addRequirements(drive);
        return command;
    }


    public Command RotateTag(String cameraName) {

        Command command = new Command() {

            @Override
            public void initialize() {
                SmartDashboard.putBoolean(
                        "rotatingTag/rotating", true);
                stopRobot();
            }

            @Override
            public void execute() {

                boolean connected =
                        limelight.hasCamera(cameraName);

                boolean seesTarget =
                        limelight.cameraHasTarget(cameraName);

                SmartDashboard.putBoolean(
                        "rotatingTag/rotating", connected);
                SmartDashboard.putBoolean(
                        "rotatingTag/rotating", seesTarget);

                if (!connected || !seesTarget) {
                    SmartDashboard.putString(
                            "rotatingTag/Status",
                            "Waiting for camera or target");
                    stopRobot();
                    return;
                }

                int tagID =
                        limelight.getCameraTagID(cameraName);

                double tx =
                        limelight.getCameraTx(cameraName);


                SmartDashboard.putNumber(
                        "rotatingTag/Tag ID", tagID);
                SmartDashboard.putNumber(
                        "rotatingTag/TX", tx);


                double rotation = 0.0;

                if (Math.abs(tx) > TX_TOLERANCE) {
                    rotation = MathUtil.clamp(
                            -tx * TURN_KP,
                            -MAX_TURN_SPEED * TX_Speed_multiplier,
                            MAX_TURN_SPEED * TX_Speed_multiplier);
                }

                SmartDashboard.putNumber(
                        "FollowTag/Rotation Command", rotation);

                SmartDashboard.putString(
                        "rotatingTag/Status",
                                Math.abs(tx) <= TX_TOLERANCE
                                ? "Target reached"
                                : "rotating tag");

                drive.driveRobotRelative(
                        new ChassisSpeeds(0, 0.0, rotation));
            }

            @Override
            public void end(boolean interrupted) {
                stopRobot();
                SmartDashboard.putBoolean(
                        "FollowTag/Running", false);
                SmartDashboard.putString(
                        "FollowTag/Status", "Command ended");
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            private void stopRobot() {
                drive.driveRobotRelative(new ChassisSpeeds());
            }
        };

        command.addRequirements(drive);
        return command;
    }

    public void registerNamedCommands(String cameraName) {
        NamedCommands.registerCommand(
                "FollowTag", followTag(cameraName));
    }
}
