package frc.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveDriveModule {

    private CANSparkMax angleMotor, driveMotor;
    private AnalogInput angleInput;
    private PIDController angleController;
    private Rotation2d offset;

    public SwerveDriveModule(
            int anglePort, int analogPort,
            Rotation2d offset,
            int drivePort,
            PIDController angleController
    ) {
        this.angleMotor = new CANSparkMax(anglePort, MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        this.angleInput = new AnalogInput(analogPort);
        this.angleController = angleController;
        this.offset = offset;
    }

    public Rotation2d getModuleHeading() {
        return new Rotation2d(
            ((1.0 - angleInput.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI) + offset.getRadians())
        );
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        // TODO
    }

    public void setTargetDrivePower(double drivePower) {
        // TODO
    }

    public void periodic() {
        // TODO
    }

}
