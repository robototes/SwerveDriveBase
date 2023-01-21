package frc.team2412.swervedrivebase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.team2412.swervedrivebase.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controls {
    public final CommandXboxController primaryController = new CommandXboxController(Constants.PRIMARY_CONTROLLER_PORT);
    //public static Joystick joystick = new Joystick(0);
    public Controls(RobotContainer r){

        CommandScheduler.getInstance().setDefaultCommand(r.drivetrainSubsystem, new DriveCommand(r.drivetrainSubsystem, primaryController::getLeftY, primaryController::getLeftX, primaryController::getRightX));

        primaryController.back().onTrue(new InstantCommand(r.drivetrainSubsystem::zeroGyroAngle));
        //();
        //new JoystickButton(joystick, 1).whenPressed(r.drivetrainSubsystem::zeroGyroAngle);

       // NEW COMMENT TEAM primaryController.getLeftBumperButton().whileTrue(new InstantCommand(()->r.drivetrainSubsystem.modifyDriveSpeed(-0.05)));
       // NEW COMMENT TEAM primaryController.getRightBumperButton().whileTrue(new InstantCommand(()->r.drivetrainSubsystem.modifyDriveSpeed(0.05)));
        //primaryController.getStartButton().whenPressed(r.drivetrainSubsystem::resetWheelAngles);

//        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(r.drivetrainSubsystem::enableFieldCentric);
//        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(r.drivetrainSubsystem::disableFieldCentric);

//        primaryController.getRightBumperButton().whenPressed(r.drivetrainSubsystem::adultMode).whenReleased(r.drivetrainSubsystem::toddlerMode);



    }


    public CommandXboxController getPrimaryController() {
        return primaryController;
    }

}
