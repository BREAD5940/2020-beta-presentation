package frc.robot.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {

    private final XboxController xbox;
    private final DriveSubsystem drive;

    public DriveCommand(DriveSubsystem drive, XboxController xboxController) {
        super();
        this.xbox = xboxController;
        addRequirements(drive);
        this.drive = drive;
    }

    @Override
    public void execute() {
        var forward = -xbox.getY(GenericHID.Hand.kRight) * 0.5;
        var strafe = -xbox.getX(GenericHID.Hand.kRight) * 0.5;
        var turn = -xbox.getX(GenericHID.Hand.kLeft) * 0.6;
//        if(Math.abs(forward) < 0.08) forward = 0.0;
//        if(Math.abs(strafe) < 0.08) strafe = 0.0;
//        if(Math.abs(turn) < 0.08) turn = 0.0;

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drive.odometry.getPoseMeters().getRotation());
        drive.setSpeeds(speeds);

    }
}
