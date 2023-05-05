package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class CommandRobot extends Robot {

    private final GamepadEx driverGamepad;
    private final GamepadEx manipulatorGamepad;

    private final Drivetrain drivetrain;

    public CommandRobot(HardwareMap hardwareMap, Gamepad driverGamepad, Gamepad manipulatorGamepad) {
        this.driverGamepad = new GamepadEx(driverGamepad);
        this.manipulatorGamepad = new GamepadEx(manipulatorGamepad);

        // instantiate subsystems & set default commands
        this.drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setDefaultCommand(drivetrain.teleopDrive(this.driverGamepad));

        register(drivetrain);

        // bind commands
        new Trigger(() -> driverGamepad.left_trigger > 0.5)
                .whenActive(() -> drivetrain.setMode(Drivetrain.DriveMode.SLOW))
                .whenInactive(() -> drivetrain.setMode(Drivetrain.DriveMode.NORMAL));

    }
}