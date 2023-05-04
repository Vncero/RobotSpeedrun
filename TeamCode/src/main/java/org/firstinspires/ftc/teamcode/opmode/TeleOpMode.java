package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.CommandRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        CommandRobot robot = new CommandRobot(hardwareMap, gamepad1, gamepad2); // this should literally be it
    }
}