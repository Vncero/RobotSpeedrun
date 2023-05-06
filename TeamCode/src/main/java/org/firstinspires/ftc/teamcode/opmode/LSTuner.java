package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandRobot;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous
@Config
public class LSTuner extends LinearOpMode {
    private CommandRobot m_robot;

    public static double position = 15;

    public static double slideKP = Constants.LinearSlide.slideKP;
    public static double flipperKP = 0;
    public static double flipperPosition = 0; 
    
    @Override
    public void runOpMode() {
        m_robot = new CommandRobot(hardwareMap, gamepad1, gamepad2, telemetry, new RunCommand(() -> {
            // add telemetry for linearslides
            m_robot.getLift().getSlide().setPositionCoefficient(slideKP);
            m_robot.getLift().getFlipper().setPositionCoefficient(slideKP);
//            telemetry.addData("flipperPos", m_robot.getLift().getFlipperPosition());
            telemetry.addData("Current Slide Position", m_robot.getLift().getSlidePosition());
            telemetry.addData("slide kp", m_robot.getLift().getSlide().getPositionCoefficient());
            telemetry.addData("flipper kp", m_robot.getLift().getSlide().getPositionCoefficient());
//            telemetry.addData("Target Position", m_robot.getPosition().getSlideHeightCentimeters());
            telemetry.addData("output", m_robot.getLift().getSlide().motor.getPower());
            telemetry.addData("Encoder pos", m_robot.getLift().getSlide().encoder.getPosition());
            telemetry.addData("flipper", m_robot.getLift().getFlipper().encoder.getPosition());
            telemetry.addData("flipper output", m_robot.getLift().getFlipper().motor.getPower());
//            telemetry.addData()
            telemetry.addData("setpoint", m_robot.getLift().targetSlide);
            telemetry.addData("slide at target: ", m_robot.getLift().slideAtPosition());
            telemetry.addData("flipper at target: ", m_robot.getLift().flipperAtPosition());

            telemetry.update();
        }).alongWith(new RunCommand(() -> m_robot.getLift().setPosition(position, flipperPosition == 1 ? Constants.LinearSlide.FlipperPosition.UP : Constants.LinearSlide.FlipperPosition.DOWN))));

//        cmd.alongWith(m_robot.getLift().toPosition(Constants.LinearSlide.Position.HIGH));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            m_robot.run();
        }
        m_robot.reset();
    }
}
