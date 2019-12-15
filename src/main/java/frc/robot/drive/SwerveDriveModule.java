package frc.robot.drive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;

public class SwerveDriveModule {

    private final CANPIDController angleCANPIDController;
    public CANSparkMax angleMotor, driveMotor;
    private CANEncoder angleEncoder, driveEncoder;
    private AnalogInput angleInput;
    private PIDController angleController;
    private Rotation2d offset;

    private static final double kEncoderClicksPerInch = (1.0 / (4.0 * Math.PI / 60.0 * 15.0 / 20.0 * 24.0 / 38.0 * 18.0));

    SwerveDriveModule(
            int anglePort, int analogPort,
            Rotation2d offset,
            int drivePort,
            PIDController angleController
    ) {
        this.angleMotor = new CANSparkMax(anglePort, MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        this.angleEncoder = angleMotor.getEncoder();
        this.driveEncoder = driveMotor.getEncoder();
        this.angleCANPIDController = angleMotor.getPIDController();

        this.angleInput = new AnalogInput(analogPort);
        this.angleController = angleController;
        this.offset = offset;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleCANPIDController.setOutputRange(-0.1, 0.1);
    }

    Rotation2d getModuleHeading() {
        return new Rotation2d(
            ((1.0 - angleInput.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI) + offset.getRadians())
        );
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        // TODO reversing but eh
        angleController.setSetpoint(targetHeading.getRadians());
    }

    public void setTargetDrivePower(double drivePower) {
        driveMotor.set(drivePower);
    }

    private double getVelocityMetersPerSec() {
        return Units.inchesToMeters(driveEncoder.getVelocity() / 60.0 * kEncoderClicksPerInch);
    }

    void periodic() {
        var anglePower = angleController.calculate(getModuleHeading().getRadians());
        angleCANPIDController.setReference(anglePower, ControlType.kDutyCycle);
    }

    SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getModuleHeading());
    }

    public void setWantedState(SwerveModuleState state) {
        setTargetHeading(state.angle);
        setTargetDrivePower(state.speedMetersPerSecond);
    }
}
