package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;

public class PickupCommand extends SequentialCommandGroup {
    public PickupCommand(org.firstinspires.ftc.teamcode.subsystems.Lift lift, Constants.Lift.Position position) {
//        super(new SetLiftPositionCommand(lift, position), new InstantCommand(lift::closeClaw), new WaitCommand(500),
//                new SetLiftPositionCommand(lift, Constants.Lift.Position.TRANSPORT));
    }
}
