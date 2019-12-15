package frc.robot.arm;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CharacterizationCommand extends CommandBase {

    private NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

    private double priorAutospeed = 0.0;
    private Number[] telemetry = new Number[6];
    private Proximal prox;

    public CharacterizationCommand(Proximal prox) {
        addRequirements(prox);
        this.prox = prox;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void execute() {
        var autoSpeed = autoSpeedEntry.getDouble(0.0);
        priorAutospeed = autoSpeed;

        System.out.println("Autospeed: " + autoSpeed);
        prox.setPower(autoSpeed);

        telemetry[0] = Timer.getFPGATimestamp();
        telemetry[1] = RobotController.getBatteryVoltage();
        telemetry[2] = autoSpeed;
        telemetry[3] = prox.getMaster().getMotorOutputVoltage();
        telemetry[4] = prox.getPositionRadians();
        telemetry[5] = prox.getVelocityRadPerSec();
        telemetryEntry.setNumberArray(telemetry);
        
    }

}