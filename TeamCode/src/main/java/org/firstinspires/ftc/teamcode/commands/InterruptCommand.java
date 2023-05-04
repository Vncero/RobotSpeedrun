package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.purepursuit.actions.InterruptAction;

public abstract class InterruptCommand extends CommandBase implements InterruptAction {
    @Override
    public void doAction() {
        schedule();
        while (!isScheduled()) {}
        while (!isFinished()) {}
    }
}
