package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.CommandRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "IDK")
public class TeleOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        CommandRobot robot = new CommandRobot(hardwareMap, gamepad1, gamepad2); // this should literally be it
    }
   @Override
    public void runOpMode() {
        // Make sure ID match configuration 
        //DcMotorEx motorFrontLeft = hardwareMap.dcMotorEx.get("frontLeft");
        //DcMotorEx motorBackLeft = hardwareMap.dcMotorEx.get("backLeft");
        //DcMotorEx motorFrontRight = hardwareMap.dcMotorEx.get("frontRight");
        //DcMotorEx motorBackRight = hardwareMap.dcMotorEx.get("backRight");

        
        DcmotorEx arm = hardwareMap.dcMotorEx.get("arm");
        Servo intake = hardwareMap.getController("intake");

        // // Reverse the right side motors
        // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //wheel motor I guess you know need it
            // double y = gamepad1.left_stick_y;
            // double x = gamepad1.left_stick_x; 
            // double rx = gamepad1.right_stick_x;

            // double frontLeftPower = (-rx + y + x);
            // double backLeftPower = (rx + y  + x);
            // double frontRightPower = (rx + y - x);
            // double backRightPower = (-rx + y - x);

            // motorFrontLeft.setPower(0.81 * frontLeftPower);
            // motorBackLeft.setPower(0.81 * backLeftPower);
            // motorFrontRight.setPower(0.81 * frontRightPower);
            // motorBackRight.setPower(0.81 * backRightPower);



            //arm
            int armticks = 0;

            while (gamepad2.dpad_up) {
             
                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(armticks);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //just to make it consistent
                if (armticks > 0)/* put however number you need */{
                    arm.setVelocity(200);
                } else {
                    arm.setVelocity(0);
                }
            while (gamepad2.dpad_down) {
               
                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(-armticks);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //just to make it consistent
                if (armticks < 0)/* put however number you need */{
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
        telemetry.addData("Front Left Motor Power", motorFrontLeft.getPower());
        telemetry.addData("Front Right Motor Power", motorFrontRight.getPower());
        telemetry.addData("Back Left Motor Power", motorBackLeft.getPower());
        telemetry.addData("Back Right Motor Power", motorBackRight.getPower());
        telemetry.addData("Arm Power", arm.getVelocity());
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("Intake Power", intake.getVelocity());
        telemetry.addData("intake position", intake.getPosition());
        telemetry.update();
    }
}
}