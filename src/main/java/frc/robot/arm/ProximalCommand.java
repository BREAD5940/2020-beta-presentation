package frc.robot.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ProximalCommand extends CommandBase {

    XboxController xbox;

    public ProximalCommand(XboxController xbox, Proximal proximal) { this.xbox = xbox; addRequirements(proximal); }

    @Override
    public void execute() {
        var power = xbox.getY(Hand.kLeft);
        System.out.println(power);
    }

}