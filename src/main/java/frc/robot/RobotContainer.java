package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.arm.CharacterizationCommand;
import frc.robot.arm.Proximal;
import frc.robot.arm.Wrist;
import frc.robot.drive.DriveSubsystem;

import java.util.List;

public class RobotContainer {

    private final Proximal proximal;
    private final Wrist wrist;
    private final XboxController xbox;
    private final DriveSubsystem drive;

    public RobotContainer() {
        this.proximal = new Proximal();
        this.wrist = new Wrist();
        drive = new DriveSubsystem();

        this.xbox = new XboxController(0);

        wrist.setDefaultCommand(new RunCommand(() -> {
            var power = xbox.getY(GenericHID.Hand.kRight);

            wrist.setPower(power / 3.0);
        }, wrist));

        proximal.setDefaultCommand(new RunCommand(() -> {
            var power = xbox.getY(GenericHID.Hand.kLeft);
        
            var pos = proximal.getMaster().getSensorCollection().getPulseWidthPosition();
            SmartDashboard.putNumber("prox rel", proximal.getPositionRadians() / Math.PI * 180.0);
            SmartDashboard.putNumber("prox abs", (pos % 4096));

            proximal.setPower(power / 3.0);
        }, proximal));

        new JoystickButton(xbox, Button.kA.value).whenPressed(new InstantCommand(() -> { proximal.resetPosition(Math.toRadians(90.0)); }));
        new JoystickButton(xbox, Button.kB.value).whenPressed(new CharacterizationCommand(proximal));
        new JoystickButton(xbox, Button.kX.value).whenPressed(new RunCommand(() -> proximal.setPositionTarget(Math.toRadians(90)), proximal));
        new JoystickButton(xbox, Button.kY.value).whenPressed(new RunCommand(() -> proximal.setPositionTarget(Math.toRadians(45)), proximal));

        new POVButton(xbox, 90).whenPressed(new RamseteCommand(
                getAutoPath(),
                drive.odometry::getPoseMeters,
                new RamseteController(2.0, 0.7),
                drive.feedforward,
                drive.kinematics,
                drive::getWheelSpeeds,
                new PIDController(1.4, 0, 0),
                new PIDController(1.4, 0, 0),
                drive::setVoltages,
                drive
        ));

    }

    private Trajectory getAutoPath() {
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d()),
                List.of(),
                new Pose2d(Units.feetToMeters(5), 0, new Rotation2d()),
                new TrajectoryConfig(Units.feetToMeters(3), Units.feetToMeters(10))
                        .setReversed(false).setStartVelocity(0).setEndVelocity(0).setKinematics(drive.kinematics)
                        .addConstraint(new DifferentialDriveVoltageConstraint(drive.feedforward, drive.kinematics, 12))
        );
    }

}
