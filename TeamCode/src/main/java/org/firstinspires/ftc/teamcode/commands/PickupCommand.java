package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class PickupCommand extends SequentialCommandGroup {
    public PickupCommand(Lift lift, Constants.LinearSlide.Position position) {
        super(new SetLiftPositionCommand(lift, position), new InstantCommand(lift::closeClaw), new WaitCommand(500),
                new SetLiftPositionCommand(lift, Constants.LinearSlide.Position.TRANSPORT));
    }
}
