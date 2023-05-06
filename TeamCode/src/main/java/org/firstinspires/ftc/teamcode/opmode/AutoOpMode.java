package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.auto.PathGenerator;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class AutoOpMode extends LinearOpMode {
    private CommandRobot m_robot;

    @Override
    public void runOpMode() {
        m_robot = new CommandRobot(hardwareMap, gamepad1, gamepad2, telemetry);

        PathGenerator.generateMecanumCommand(m_robot.getDrivetrain(),
                Arrays.asList(new Pose2d(0, 0, new Rotation2d()), new Pose2d(0, 2, new Rotation2d())), false
        ).schedule();

//        cmd.alongWith(m_robot.getLift().toPosition(Constants.LinearSlide.Position.HIGH));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            m_robot.run();
        }
        m_robot.reset();
    }
}
