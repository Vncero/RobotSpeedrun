package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(Lift lift, Constants.Lift.Position position) {
//        super(new SetLiftPositionCommand(lift, position), new InstantCommand(lift::openClaw), new WaitCommand(500));
    }
}