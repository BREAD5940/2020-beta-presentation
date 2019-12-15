package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));
    public AHRS gyro = new AHRS(SPI.Port.kMXP);
    public DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(kinematics, getHeading());

    private Encoder leftEncoder = new Encoder(1, 0);
    private Encoder rightEncoder = new Encoder(1, 0);

    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.2, 1.5, 0.1);

    public TalonSRX left = new TalonSRX(0);
    public TalonSRX right = new TalonSRX(1);

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getFusedHeading());
    }

    public DriveSubsystem() {
        leftEncoder.setDistancePerPulse(Math.PI * Units.inchesToMeters(4) / 4096);
        rightEncoder.setDistancePerPulse(Math.PI * Units.inchesToMeters(4) / 4096);
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), getWheelSpeeds());
        SmartDashboard.putString("pose", odometry.getPoseMeters().toString());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void setVoltages(double leftVolts, double rightVolts) {
        var volts = RobotController.getBatteryVoltage();
        left.set(ControlMode.PercentOutput, leftVolts / volts);
        right.set(ControlMode.PercentOutput, rightVolts / volts);
    }

}
