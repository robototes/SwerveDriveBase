package org.frcteam2910.mk3.commands.autonomous;

import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.mk3.commands.DriveCommand;
import org.frcteam2910.mk3.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedTaxiAutoCommand extends SequentialCommandGroup {
    public static Axis constantAxis(double value) {
        return new Axis() {
            @Override
            public double getRaw() {
                return value;
            }
        };
    }

    public TimedTaxiAutoCommand(DrivetrainSubsystem drivebaseSubsystem) {
        addCommands(new WaitCommand(1)
                    .deadlineWith(new DriveCommand(drivebaseSubsystem, constantAxis(-0.4), constantAxis(0), constantAxis(0), false)));
    }
}
