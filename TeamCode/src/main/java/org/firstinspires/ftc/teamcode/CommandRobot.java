package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import org.firstinspires.ftc.teamcode.Constants.Lift.Position;

public class CommandRobot extends Robot {

    private final GamepadEx driverGamepad;
    private final GamepadEx manipulatorGamepad;

    private final Drivetrain drivetrain;
    private final org.firstinspires.ftc.teamcode.subsystems.Lift lift;

    private final Telemetry telemetry;

    public CommandRobot(HardwareMap hardwareMap, Gamepad driverGamepad, Gamepad manipGamepad, Telemetry telemetry) {
        this.driverGamepad = new GamepadEx(driverGamepad);
        this.manipulatorGamepad = new GamepadEx(manipGamepad);
        this.telemetry = telemetry;

        // instantiate subsystems & set default commands
        this.drivetrain = new Drivetrain(hardwareMap, telemetry);
        this.lift = new Lift(hardwareMap, telemetry);

        register(drivetrain, lift);

        drivetrain.setDefaultCommand(drivetrain.teleopDrive(this.driverGamepad));
//        lift.setDefaultCommand(new RunCommand(() -> {
////            telemetry.addData("power", manipulatorGamepad.left_stick_y);
////            telemetry.update();
//            lift.power = manipulatorGamepad.left_stick_y;
//        }, lift));

        // bind commands
        // drivetrain
        new Trigger(() -> driverGamepad.left_trigger > 0.5)
                .whenActive(() -> drivetrain.setMode(Drivetrain.DriveMode.SLOW))
                .whenInactive(() -> drivetrain.setMode(Drivetrain.DriveMode.NORMAL));
        // lift
        this.manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> lift.setPosition(Position.GROUND));
//        this.manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> lift.setPosition(Position.TRANSPORT));
        this.manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> lift.setPosition(Position.LOW));
        this.manipulatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> lift.setPosition(Position.MID));
        this.manipulatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> lift.setPosition(Position.HIGH));
        // flipper
        this.manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                () -> lift.controller.setSetPoint(Constants.Lift.FlipperPosition.UP.getPosition()),
                () -> lift.controller.setSetPoint(Constants.Lift.FlipperPosition.DOWN.getPosition()));

        // intake
        new Trigger(() -> manipGamepad.left_trigger > 0.5).whenActive(lift::openClaw);
        new Trigger(() -> manipGamepad.right_trigger > 0.5).whenActive(lift::closeClaw);
    }

    public CommandRobot(HardwareMap hardwareMap, Gamepad driverGamepad, Gamepad manipGamepad, Telemetry telemetry, Command commandToRun) {
        this(hardwareMap, driverGamepad, manipGamepad, telemetry);
        commandToRun.schedule();
    }

    public Lift getLift() {
        return this.lift;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
}