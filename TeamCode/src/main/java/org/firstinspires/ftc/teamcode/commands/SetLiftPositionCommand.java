package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.Set;

public class SetLiftPositionCommand extends CommandBase {

    private Lift lift;
    private double lsPos;
    private Constants.LinearSlide.FlipperPosition flipperPosition;

    public SetLiftPositionCommand(Lift lift, Constants.LinearSlide.Position position) {
        this(lift, position.getSlideHeightCentimeters(), position.getFlipperPosition());
    }

    public SetLiftPositionCommand(Lift lift, double ls, Constants.LinearSlide.FlipperPosition flipperPosition) {
        this.lift = lift;
        this.lsPos = ls;
        this.flipperPosition = flipperPosition;
    }

    @Override
    public void initialize() {
        this.lift.setPosition(this.lsPos, this.flipperPosition);
    }

    @Override
    public boolean isFinished() {
        return this.lift.slideAtPosition() && this.lift.flipperAtPosition();
    }
}
