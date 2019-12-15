package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.arm.CharacterizationCommand;
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
        }, wrist));

        proximal.setDefaultCommand(new RunCommand(() -> {
            var power = xbox.getY(GenericHID.Hand.kLeft);
        
            var pos = proximal.getMaster().getSensorCollection().getPulseWidthPosition();
            SmartDashboard.putNumber("prox rel", proximal.getPositionRadians() / Math.PI * 180.0);
            SmartDashboard.putNumber("prox abs", (pos % 4096));

            proximal.setPower(power / 3.0);
        }, proximal));

        new JoystickButton(xbox, Button.kA.value).whenPressed(new InstantCommand(() -> { proximal.resetPosition(Math.toRadians(90.0)); }));
        new JoystickButton(xbox, Button.kB.value).whenPressed(new CharacterizationCommand(proximal));
        new JoystickButton(xbox, Button.kX.value).whenPressed(new RunCommand(() -> proximal.setPositionTarget(Math.toRadians(90)), proximal));
        new JoystickButton(xbox, Button.kY.value).whenPressed(new RunCommand(() -> proximal.setPositionTarget(Math.toRadians(0)), proximal));


    }

}
