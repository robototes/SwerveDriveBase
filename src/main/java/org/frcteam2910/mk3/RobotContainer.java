package org.frcteam2910.mk3;

import edu.wpi.first.wpilibj2.command.*;
import org.frcteam2910.mk3.commands.DriveCommand;
import org.frcteam2910.mk3.subsystems.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;

public class RobotContainer {

    public final DrivetrainSubsystem drivetrainSubsystem;

    public RobotContainer(){
        drivetrainSubsystem = new DrivetrainSubsystem();
    }

}
