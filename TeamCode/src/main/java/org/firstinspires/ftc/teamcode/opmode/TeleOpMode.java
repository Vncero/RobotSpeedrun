package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.CommandRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name ="IDK")
public class TeleOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        CommandRobot robot = new CommandRobot(hardwareMap, gamepad1, gamepad2); // this should literally be it
    }
    @Override
    public void runOpMode() {


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //arm
            int armticks = 0;/* put however number you need */

            while (gamepad2.dpad_up) {
             
                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(armticks);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //just to make it consistent
                if (armticks > 0) {
                    arm.setVelocity(200);
                } else {
                    arm.setVelocity(0);
                }
            while (gamepad2.dpad_down) {
               
                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(-armticks);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //just to make it consistent
                if (armticks < 0){
                    arm.setVelocity(-200);
                } else {
                    arm.setVelocity(0);
                }
            }


//             //intake servo
            double Intakeposition = 0;
            double intakenewposition = 20;

            intake.setPosition(Intakeposition);

            while (gamepad2.b) {
                intake.setVelocity(intakenewposition);
            }

            while (gamepad2.a) {
                
                intake.setVelocity(-intakenewposition);
            }

            TelemetryUpdate();
        }
    }

    public void TelemetryUpdate() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Arm Power", arm.getVelocity());
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("Intake Power", intake.getVelocity());
        telemetry.addData("intake position", intake.getPosition());
        telemetry.update();
    }
}
}
