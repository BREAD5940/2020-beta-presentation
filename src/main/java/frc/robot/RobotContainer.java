package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.arm.Proximal;
import frc.robot.arm.Wrist;

public class RobotContainer {

    private final Proximal proximal;
    private final Wrist wrist;
    private final XboxController xbox;

    public RobotContainer() {
        this.proximal = new Proximal();
        this.wrist = new Wrist();

        this.xbox = new XboxController(0);

        wrist.setDefaultCommand(new RunCommand(() -> {
            var power = xbox.getY(GenericHID.Hand.kRight);
            wrist.setPower(power / 3.0);
        }));

        proximal.setDefaultCommand(new RunCommand(() -> {
            var power = xbox.getY(GenericHID.Hand.kLeft);
            proximal.setPower(power / 3.0);
        }));
    }

}
