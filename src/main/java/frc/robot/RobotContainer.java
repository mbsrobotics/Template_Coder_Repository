
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.HashSet;
import java.util.Set;

public class RobotContainer {

    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    private final LimelightFunctions limelightFunctions =
        new LimelightFunctions();

    private final LimelightAutos limelightAutos =
        new LimelightAutos(limelightFunctions, m_robotDrive);

    private final XboxController m_driverController =
            new XboxController(OIConstants.kDriverControllerPort);

    private final SendableChooser<Command> autoChooser;

    private final Set<String> publishedCameras = new HashSet<>();

    public RobotContainer() {

        System.out.println(
        "Detected Limelights: "
        + limelightFunctions.getCameraNames());

        // Build your existing PathPlanner auto list.
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> stream.filter(
                auto -> !auto.getName().equals("ExampleAuto")
        )
        );

        

        // Add Follow Tag as a standalone selectable option.
        autoChooser.addOption(
                "Follow Tag",
                limelightAutos.followTag("limelight-three"));

        autoChooser.addOption(
                "Rotate Tag",
                limelightAutos.RotateTag("limelight-two"));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putData("Limelight Camera List", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Limelight Camera List");

                builder.addDoubleProperty(
                        "Camera Count",
                        () -> limelightFunctions.getCameraCount(),
                        null);
            }
        });

        publishLimelightWidgets();

        configureButtonBindings();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(
                                        m_driverController.getLeftY(),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                        m_driverController.getLeftX(),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                        m_driverController.getRightX(),
                                        OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));
    }

    public void publishLimelightWidgets() {
        for (String cameraName : limelightFunctions.getCameraNames()) {
            if (publishedCameras.contains(cameraName)) {
                continue;
            }

            publishedCameras.add(cameraName);

            SmartDashboard.putData(
                    "Limelight Monitor/" + cameraName,
                    new Sendable() {
                        @Override
                        public void initSendable(SendableBuilder builder) {
                            builder.setSmartDashboardType("Limelight");

                            builder.addBooleanProperty(
                                    "Connected",
                                    () -> limelightFunctions.hasCamera(cameraName),
                                    null);

                            builder.addBooleanProperty(
                                    "Has Target",
                                    () -> limelightFunctions.cameraHasTarget(cameraName),
                                    null);

                            builder.addDoubleProperty(
                                    "Tag ID",
                                    () -> limelightFunctions.getCameraTagID(cameraName),
                                    null);

                            builder.addDoubleProperty(
                                    "TX",
                                    () -> limelightFunctions.getCameraTx(cameraName),
                                    null);

                            builder.addDoubleProperty(
                                    "TY",
                                    () -> limelightFunctions.getCameraTy(cameraName),
                                    null);

                            builder.addDoubleProperty(
                                    "Forward Distance (m)",
                                    () -> limelightFunctions
                                            .getCameraForwardDistanceMeters(cameraName),
                                    null);

                            builder.addDoubleProperty(
                                    "Forward Distance (ft)",
                                    () -> limelightFunctions
                                            .getCameraForwardDistanceFeet(cameraName),
                                    null);

                            builder.addDoubleProperty(
                                    "Visible Tag Count",
                                    () -> limelightFunctions
                                            .getCameraVisibleTagCount(cameraName),
                                    null);
                        }
                    });
        }
    }

    private void configureButtonBindings() {

        new JoystickButton(
                m_driverController,
                XboxController.Button.kRightBumper.value)
                .whileTrue(
                        new RunCommand(
                                () -> m_robotDrive.setX(),
                                m_robotDrive));

        new JoystickButton(
                m_driverController,
                XboxController.Button.kLeftBumper.value)
                .onTrue(
                        new InstantCommand(
                                () -> m_robotDrive.zeroHeading(),
                                m_robotDrive));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
