package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ProfiledPIDCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Lift extends SubsystemBase {
    private MotorEx slide;

    private ServoEx flipper;

    private MotorEx intake;

    private ProfiledPIDController controller;

    public Lift(HardwareMap hardwareMap) {
        this.slide = new MotorEx(hardwareMap, "slide", Motor.GoBILDA.RPM_435);
        this.flipper = new SimpleServo(hardwareMap, "flipper", 0, 1);
        this.intake = new MotorEx(hardwareMap, "intake");

        slide.setDistancePerPulse(Constants.LinearSlide.metersPerTick);
    }

    public Command toPosition(Position position) {
        return new ProfiledPIDCommand(
                this.controller,
                this.slide::getDistance,
                position::getSlideHeightMeters,
                (output, state) -> this.slide.set(MathUtils.clamp(output, -1, 1)),
                this).alongWith(new InstantCommand(() -> this.flipper.setPosition(position.flipperPosition)));
    }

    public enum Position {
        HIGH(),
        MID(),
        LOW(),
        GROUND();

        private double slideHeightMeters;
        private double flipperPosition;

        Position(double slideHeightMeters, double flipperPosition) {
            this.slideHeightMeters = slideHeightMeters;
            this.flipperPosition = flipperPosition;
        }

        public double getSlideHeightMeters() {
            return this.slideHeightMeters;
        }
    }
}