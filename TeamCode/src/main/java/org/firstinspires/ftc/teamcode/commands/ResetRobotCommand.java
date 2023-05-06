package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class ResetRobotCommand extends CommandBase {
    private Drivetrain drivetrain;
    private Lift lift;
    public ResetRobotCommand(Drivetrain drivetrain, Lift lift) {
        this.drivetrain = drivetrain;
        this.lift = lift;
    }

    @Override
    public void execute() {
        this.drivetrain.getGyro().resetYaw();
        this.lift.resetLift();
    }
}
