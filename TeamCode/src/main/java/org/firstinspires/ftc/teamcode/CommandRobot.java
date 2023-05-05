package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class CommandRobot extends Robot {

    private final GamepadEx driverGamepad;
    private final GamepadEx manipulatorGamepad;

    private final Drivetrain drivetrain;
    private final Lift lift;

    public CommandRobot(HardwareMap hardwareMap, Gamepad driverGamepad, Gamepad manipulatorGamepad) {
        this.driverGamepad = new GamepadEx(driverGamepad);
        this.manipulatorGamepad = new GamepadEx(manipulatorGamepad);

        // instantiate subsystems & set default commands
        this.drivetrain = new Drivetrain(hardwareMap);
        this.lift = new Lift(hardwareMap);

        register(drivetrain, lift);

        drivetrain.setDefaultCommand(drivetrain.teleopDrive(this.driverGamepad));

        // bind commands
        new Trigger(() -> driverGamepad.left_trigger > 0.5)
                .whenActive(() -> drivetrain.setMode(Drivetrain.DriveMode.SLOW))
                .whenInactive(() -> drivetrain.setMode(Drivetrain.DriveMode.NORMAL));
    }
}