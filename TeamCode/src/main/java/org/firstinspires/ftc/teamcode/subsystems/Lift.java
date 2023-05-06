package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ProfiledPIDCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.Supplier;

public class Lift extends SubsystemBase {
    private final MotorEx slide;

    private final MotorEx flipper;

//    private final ServoEx intake;

    public double power;

    private final ProfiledPIDController controller;

    private Telemetry telemetry;

    public double targetSlide = 0;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.slide = new MotorEx(hardwareMap, "slide", Motor.GoBILDA.RPM_435);
        this.flipper = new MotorEx(hardwareMap, "flipper", Motor.GoBILDA.RPM_312);
        this.telemetry = telemetry;
//        this.intake = new SimpleServo(hardwareMap, "intake", 0, 360);

        slide.setDistancePerPulse(Constants.LinearSlide.metersPerTick*100);
//        flipper.setDistancePerPulse()

        this.slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flipper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slide.setPositionTolerance(25); // ticks
        flipper.setPositionTolerance(25); // ticks

        slide.setPositionCoefficient(Constants.LinearSlide.slideKP);

        this.controller = new ProfiledPIDController(
                Constants.LinearSlide.slideKP,
                Constants.LinearSlide.slideKI,
                Constants.LinearSlide.slideKD,
                Constants.LinearSlide.constraints);

        this.slide.setInverted(false);

        this.slide.setRunMode(Motor.RunMode.PositionControl);

        this.flipper.setRunMode(Motor.RunMode.PositionControl);
        this.flipper.setPositionCoefficient(Constants.LinearSlide.flipperKP);
    }

    public double getSlidePosition() {
        return this.slide.getDistance();
    }

    public double getFlipperPosition() {
        return this.flipper.getDistance();
    }

    public Command toPosition(Constants.LinearSlide.Position position) {
        return toPosition(position.getSlideHeightCentimeters(), position.getFlipperPosition());
    }

    public Command toPosition(double slide, Constants.LinearSlide.FlipperPosition flipperPosition) {
        return new ProfiledPIDCommand(
                this.controller,
                this.slide::getDistance,
                slide,
                (output, state) -> {
                    telemetry.addData("error", controller.getPositionError());
                    telemetry.addData("output", output);
                    this.slide.set(MathUtils.clamp(output, -0.5, 0.5));
                },
                this).alongWith(new InstantCommand(() -> {
            this.flipper.setTargetDistance(flipperPosition.getPosition());
//            this.flipper.set(0.2);
        })).interruptOn(this.flipper::atTargetPosition).whenFinished(() -> this.flipper.set(0));
    }

    public void moveIntake(double position) {
//        intake.setPosition(position);
    }

    public void closeClaw() {

    }

    public void openClaw() {}

    public void setPosition(Constants.LinearSlide.Position position) {
        setPosition(position.getSlideHeightCentimeters(), position.getFlipperPosition());
    }

    public void setPosition(double slide, Constants.LinearSlide.FlipperPosition flipperPosition) {
        targetSlide = slide;
        this.slide.setTargetDistance(slide);
        this.flipper.setTargetDistance(flipperPosition.getPosition());
    }

    public void resetLift() {
        this.slide.resetEncoder();
        this.flipper.resetEncoder();
    }

    public MotorEx getSlide() {
        return slide;
    }

    public MotorEx getFlipper() {
        return flipper;
    }

    public boolean slideAtPosition() {
        return this.slide.atTargetPosition();
    }

    public boolean flipperAtPosition() {
        return this.flipper.atTargetPosition();
    }

    @Override
    public void periodic() {
        slide.set(0.2);
        flipper.set(0.2);
    }
}