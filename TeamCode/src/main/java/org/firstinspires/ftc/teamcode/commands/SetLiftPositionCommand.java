package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;

public class SetLiftPositionCommand extends CommandBase {

    private org.firstinspires.ftc.teamcode.subsystems.Lift lift;
    private double lsPos;
    private Constants.Lift.FlipperPosition flipperPosition;

    public SetLiftPositionCommand(org.firstinspires.ftc.teamcode.subsystems.Lift lift, Constants.Lift.Position position) {
        this(lift, position.getSlideHeightCentimeters(), position.getFlipperPosition());
    }

    public SetLiftPositionCommand(org.firstinspires.ftc.teamcode.subsystems.Lift lift, double ls, Constants.Lift.FlipperPosition flipperPosition) {
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
