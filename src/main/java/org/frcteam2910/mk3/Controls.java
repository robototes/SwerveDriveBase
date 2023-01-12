package org.frcteam2910.mk3;

import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.mk3.commands.DriveCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controls {
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    //public static Joystick joystick = new Joystick(0);
    public Controls(RobotContainer r){

        CommandScheduler.getInstance().setDefaultCommand(r.drivetrainSubsystem, new DriveCommand(r.drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));

        primaryController.getBackButton().whileTrue(new InstantCommand(r.drivetrainSubsystem::zeroGyroAngle));
        //new JoystickButton(joystick, 1).whenPressed(r.drivetrainSubsystem::zeroGyroAngle);

        primaryController.getLeftBumperButton().whileTrue(new InstantCommand(()->r.drivetrainSubsystem.modifyDriveSpeed(-0.05)));
        primaryController.getRightBumperButton().whileTrue(new InstantCommand(()->r.drivetrainSubsystem.modifyDriveSpeed(0.05)));
        //primaryController.getStartButton().whenPressed(r.drivetrainSubsystem::resetWheelAngles);

//        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(r.drivetrainSubsystem::enableFieldCentric);
//        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(r.drivetrainSubsystem::disableFieldCentric);

//        primaryController.getRightBumperButton().whenPressed(r.drivetrainSubsystem::adultMode).whenReleased(r.drivetrainSubsystem::toddlerMode);

    }





    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

}
