package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class CommandRobot extends Robot {

    private final GamepadEx driverGamepad;
    private final GamepadEx manipulatorGamepad;

    private final Drivetrain drivetrain;
    private final Lift lift;

    private Constants.LinearSlide.Position position;

    private Telemetry telemetry;

    public CommandRobot(HardwareMap hardwareMap, Gamepad driverGamepad, Gamepad manipulatorGamepad, Telemetry telemetry) {
        this.driverGamepad = new GamepadEx(driverGamepad);
        this.manipulatorGamepad = new GamepadEx(manipulatorGamepad);
        this.telemetry = telemetry;

        this.position = Constants.LinearSlide.Position.GROUND;

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
        new Trigger(() -> driverGamepad.left_trigger > 0.5)
                .whenActive(() -> drivetrain.setMode(Drivetrain.DriveMode.SLOW))
                .whenInactive(() -> drivetrain.setMode(Drivetrain.DriveMode.NORMAL));

        new Trigger(() -> driverGamepad.a).whenActive(new InstantCommand(() -> lift.setPosition(Constants.LinearSlide.Position.GROUND)));
        new Trigger(() -> driverGamepad.b).whenActive(new InstantCommand(() -> lift.setPosition(Constants.LinearSlide.Position.LOW)));
        new Trigger(() -> driverGamepad.x).whenActive(new InstantCommand(() -> lift.setPosition(Constants.LinearSlide.Position.MID)));
        new Trigger(() -> driverGamepad.y).whenActive(new InstantCommand(() -> lift.setPosition(Constants.LinearSlide.Position.HIGH)));

//        this.driverGamepad.getGamepadButton(GamepadKeys.Button.A)
//                .toggleWhenPressed(() -> this.position = Constants.LinearSlide.Position.GROUND, () -> this.position = Constants.LinearSlide.Position.HIGH);
    }

    public CommandRobot(HardwareMap hardwareMap, Gamepad driverGamepad, Gamepad manipGamepad, Telemetry telemetry, Command commandToRun) {
        this(hardwareMap, driverGamepad, manipGamepad, telemetry);
        commandToRun.schedule();
    }

    public Lift getLift() {
        return this.lift;
    }

    public Constants.LinearSlide.Position getPosition() {
        return this.position;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
}