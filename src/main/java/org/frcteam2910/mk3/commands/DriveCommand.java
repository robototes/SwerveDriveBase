package org.frcteam2910.mk3.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.mk3.subsystems.DrivetrainSubsystem;
import org.frcteam2910.mk3.Controls;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Axis forward;
    private final Axis strafe;
    private final Axis rotation;
    private final boolean fieldOriented;

    private static boolean USE_JOYSTICK = false;

    public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation, boolean fieldOriented) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (USE_JOYSTICK) {
            //Joystick joystick = Controls.joystick;
            //drivetrainSubsystem.drive(new Vector2(-joystick.getX(), joystick.getY()), Math.pow(joystick.getTwist(), 2)/3, fieldOriented);
        } else {
            drivetrainSubsystem.drive(new Vector2(-forward.get(false), strafe.get(false)), rotation.get(false), fieldOriented);
        }
//        if(drivetrainSubsystem.isFieldOriented()) System.out.println(drivetrainSubsystem.getGyroAngle());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, true);
    }
}
