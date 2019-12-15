package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public class DriveSubsystem extends SubsystemBase {

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    Rotation2d getGyro() {
        return new Rotation2d(-gyro.getFusedHeading());
    }

    private SwerveDriveModule flModule = new SwerveDriveModule(
        2, 2, Rotation2d.fromDegrees(142 + 73), 1,
            new PIDController(0.5, 0.0, 0.0)
    );
    private SwerveDriveModule frModule = new SwerveDriveModule(
        4, 1, Rotation2d.fromDegrees(87), 3,
            new PIDController(0.5, 0.0, 0.0)
    );
    private SwerveDriveModule blModule = new SwerveDriveModule(
        8, 0, Rotation2d.fromDegrees(92 - 25 - 3), 7,
            new PIDController(0.5, 0.0, 0.0)
    );
    private SwerveDriveModule brModule = new SwerveDriveModule(
        6, 3, Rotation2d.fromDegrees(39), 5,
            new PIDController(0.5, 0.0, 0.0)
    );

    public DriveSubsystem() {
        flModule.driveMotor.setInverted(false);
        frModule.driveMotor.setInverted(true);
        blModule.driveMotor.setInverted(false);
        brModule.driveMotor.setInverted(true);
        modules.forEach(m -> m.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake));
        modules.forEach(m -> m.angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake));
    }

    private double baseWidth = Units.inchesToMeters(24);
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(baseWidth / 2.0, baseWidth / 2.0),
            new Translation2d(baseWidth / 2.0, -baseWidth / 2.0),
            new Translation2d(-baseWidth / 2.0, -baseWidth / 2.0),
            new Translation2d(-baseWidth / 2.0, baseWidth / 2.0)
    );

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyro());

    public List<SwerveDriveModule> modules = List.of(flModule, frModule, blModule, brModule);


    @Override
    public void periodic() {
         modules.forEach(SwerveDriveModule::periodic);
         odometry.update(getGyro(), flModule.getState(), frModule.getState(),
                 brModule.getState(), blModule.getState());

        SmartDashboard.putNumber("flAngle", flModule.getModuleHeading().getDegrees());
        SmartDashboard.putNumber("frAngle", frModule.getModuleHeading().getDegrees());
        SmartDashboard.putNumber("blAngle", blModule.getModuleHeading().getDegrees());
        SmartDashboard.putNumber("brAngle", brModule.getModuleHeading().getDegrees());
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        var states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);
        for(int i = 0; i < 4; i++) {
            modules.get(i).setWantedState(states[i]);
        }
    }
}
