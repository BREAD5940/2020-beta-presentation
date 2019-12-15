package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Proximal extends SubsystemBase {

    private final TalonSRX master = new TalonSRX(30);
    private final TalonSRX follower = new TalonSRX(31);

    public void setPower(double power) {
        master.set(ControlMode.PercentOutput, power);
    }

    public TalonSRX getMaster() {
        return master;
    }

    public Proximal() {
        master.configFactoryDefault();
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        master.setSensorPhase(true);

        follower.configFactoryDefault();
        follower.follow(master);
        follower.setInverted(InvertType.OpposeMaster);

        // Put PID stuff here
    }

    public double getPositionRadians() {
        double currentPosition = master.getSelectedSensorPosition();
        currentPosition /= 9.33;
        currentPosition /= 4096.0;
        currentPosition *= 2 * Math.PI;
        return currentPosition;
    }

    public double getVelocityRadPerSec() {
        var currentVelocity = master.getSelectedSensorVelocity() * 10.0;
        currentVelocity /= 9.33;
        currentVelocity /= 4096.0;
        currentVelocity *= 2 * Math.PI;
        return currentVelocity;
    }

    public void resetPosition(double positionRad) {
        var position = positionRad / Math.PI / 2.0;
        position *= 4096.0;
        position *= 9.33;
        master.setSelectedSensorPosition((int) position);
    }

    public void setPositionTarget(double positionTarget) {
        this.positionTarget = positionTarget;
    }

    private double positionTarget;
    private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
    private PIDController controller = new PIDController(0, 0, 0);
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Math.toRadians(10), Math.toRadians(10));

    @Override
    public void periodic() {
        var currentState = new TrapezoidProfile.State(getPositionRadians(), getVelocityRadPerSec());
        var profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(positionTarget, 0.0), currentState);
        var goal = profile.calculate(0.020);
        var ff = feedforward.calculate(currentState.position, goal.position, goal.velocity);
        var fb = controller.calculate(currentState.position, goal.position);
    }

}
