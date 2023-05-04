package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.CommandRobot;

public class TeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        CommandRobot robot = new CommandRobot(hardwareMap, gamepad1, gamepad2); // this should literally be it
    }
}