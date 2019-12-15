package frc.robot.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Proximal extends SubsystemBase {

    private final TalonSRX master = new TalonSRX(30);
    private final TalonSRX follower = new TalonSRX(31);

    public void setPower(double power) {
        master.set(ControlMode.PercentOutput, power);
    }

    public Proximal() {
        master.configFactoryDefault();
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        master.setSensorPhase(true);

        follower.configFactoryDefault();
        follower.follow(master);

        // Put PID stuff here
    }

}
