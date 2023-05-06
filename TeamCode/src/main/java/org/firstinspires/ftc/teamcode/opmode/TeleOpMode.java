package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.CommandRobot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMode extends LinearOpMode {
    private CommandRobot m_robot;

    @Override
    public void runOpMode() {
        m_robot = new CommandRobot(hardwareMap, gamepad1, gamepad2, telemetry);

        m_robot.getDrivetrain().setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            m_robot.run();
//            telemetry.addData("flipper target position", m_robot.getLift());
//            telemetry.addData("flipper kP: ", m_robot.getLift().getFlipper().getPositionCoefficient());
//            telemetry.addData("flipper output: ", m_robot.getLift().getFlipper().motor.getPower());
//            telemetry.addData("flipper encoder ticks: ", m_robot.getLift().getFlipper().encoder.getPosition());
//            telemetry.addData("bumper", gamepad1.left_trigger);
            telemetry.addData("flipper setpoint", m_robot.getLift().controller.getSetPoint());
            telemetry.addData("flipper error", m_robot.getLift().controller.getPositionError());
            telemetry.update();
        }
        m_robot.reset();
    }
}