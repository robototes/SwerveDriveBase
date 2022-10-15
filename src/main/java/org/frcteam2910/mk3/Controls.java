package org.frcteam2910.mk3;

import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.mk3.commands.DriveCommand;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Controls {
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    public Controls(RobotContainer r){

        CommandScheduler.getInstance().setDefaultCommand(r.drivetrainSubsystem, new DriveCommand(r.drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));

        primaryController.getBackButton().whenPressed(r.drivetrainSubsystem::zeroGyroAngle);

        primaryController.getLeftBumperButton().whileHeld(()->r.drivetrainSubsystem.modifyDriveSpeed(-0.05));
        primaryController.getRightBumperButton().whileHeld(()->r.drivetrainSubsystem.modifyDriveSpeed(0.05));
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
