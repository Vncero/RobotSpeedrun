package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.CommandRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMode extends LinearOpMode {
    private CommandRobot m_robot;


    @Override
    public void runOpMode() {
        m_robot = new CommandRobot(hardwareMap, gamepad1, gamepad2, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            m_robot.run();
            telemetry.update();
        }
        m_robot.reset();
    }
}