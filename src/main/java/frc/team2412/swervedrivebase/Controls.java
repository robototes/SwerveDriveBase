package frc.team2412.swervedrivebase;

import edu.wpi.first.wpilibj.XboxController;
import frc.team2412.swervedrivebase.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Controls {
    public final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    //public static Joystick joystick = new Joystick(0);
    public Controls(RobotContainer r){

        CommandScheduler.getInstance().setDefaultCommand(r.drivetrainSubsystem, new DriveCommand(r.drivetrainSubsystem, primaryController::getLeftY, primaryController::getLeftX, primaryController::getRightX));

       // NEW COMMENT TEAM primaryController.getBackButton().whileTrue(new InstantCommand(r.drivetrainSubsystem::zeroGyroAngle));
        //new JoystickButton(joystick, 1).whenPressed(r.drivetrainSubsystem::zeroGyroAngle);

       // NEW COMMENT TEAM primaryController.getLeftBumperButton().whileTrue(new InstantCommand(()->r.drivetrainSubsystem.modifyDriveSpeed(-0.05)));
       // NEW COMMENT TEAM primaryController.getRightBumperButton().whileTrue(new InstantCommand(()->r.drivetrainSubsystem.modifyDriveSpeed(0.05)));
        //primaryController.getStartButton().whenPressed(r.drivetrainSubsystem::resetWheelAngles);

//        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(r.drivetrainSubsystem::enableFieldCentric);
//        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(r.drivetrainSubsystem::disableFieldCentric);

//        primaryController.getRightBumperButton().whenPressed(r.drivetrainSubsystem::adultMode).whenReleased(r.drivetrainSubsystem::toddlerMode);

    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

}
