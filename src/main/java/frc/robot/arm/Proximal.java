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
        wantsPos = false;
    }

    public TalonSRX getMaster() {
        return master;
    }

    public Proximal() {
        master.configFactoryDefault();
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        master.setSensorPhase(true);
        master.configVoltageCompSaturation(12.0);
        master.enableVoltageCompensation(true);

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
        this.wantsPos = true;
    }

    private double positionTarget;
    private boolean wantsPos = false;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 4.0, 0.118);
    private PIDController controller = new PIDController(1.3, 0, 0.35);
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Math.toRadians(50), Math.toRadians(300));

    @Override
    public void periodic() {
        if(!wantsPos) return;

        // var angle = getPositionRadians();

        var currentState = new TrapezoidProfile.State(getPositionRadians(), getVelocityRadPerSec());
        var profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(positionTarget, 0.0), currentState);
        var goal = profile.calculate(0.040);
        var ff = feedforward.calculate(goal.velocity, 0.0);
        var fb = controller.calculate(currentState.position, goal.position);
        var cos = Math.cos(currentState.position) * 0.25;
        var sum = fb + ff + cos;

        System.out.println("sum " + sum + "fb: " + fb + " cos: " + cos);

        master.configPeakOutputForward(0.2);
        master.configPeakOutputReverse(-0.2);
        master.set(ControlMode.PercentOutput, sum / 12.0);
    }

}
