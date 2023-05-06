package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Lift extends SubsystemBase {
    private final MotorEx slide;

    private final MotorEx flipper;

    private final Servo intake;

    public double power;

    public final PIDFController controller;

    private final Telemetry telemetry;

    public double targetSlide = 0;

    public double flipperSetpoint = 0;

    public double flipperPower = -0.03;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.slide = new MotorEx(hardwareMap, "slide", Motor.GoBILDA.RPM_435);
        this.flipper = new MotorEx(hardwareMap, "flipper", Motor.GoBILDA.RPM_312);
        this.telemetry = telemetry;
        this.intake = hardwareMap.get(Servo.class, "intake");
        this.controller = new PIDFController(Constants.Lift.flipperKP, 0, 0,  Constants.Lift.flipperKFF);
        this.slide.setDistancePerPulse(Constants.Lift.metersPerTick*100);
//        flipper.setDistancePerPulse()

        this.slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flipper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.slide.setPositionTolerance(25); // ticks
        this.controller.setTolerance(25); // ticks

        this.slide.setPositionCoefficient(Constants.Lift.slideKP);

        this.slide.setInverted(false);

        this.slide.setRunMode(Motor.RunMode.PositionControl);

//        this.flipper.setRunMode(Motor.RunMode.PositionControl);
//        this.flipper.setPositionCoefficient(Constants.LinearSlide.flipperKP);
//        this.flipper.setFeedforwardCoefficients();
        this.flipper.resetEncoder();
        this.setPosition(0, Constants.Lift.FlipperPosition.DOWN);
    }

    public double getSlidePosition() {
        return this.slide.getDistance();
    }

    public double getFlipperPosition() {
        return this.flipper.getDistance();
    }

//    public Command toPosition(Constants.LinearSlide.Position position) {
//        return toPosition(position.getSlideHeightCentimeters(), position.getFlipperPosition());
//    }

//    public Command toPosition(double slide, Constants.LinearSlide.FlipperPosition flipperPosition) {
//        return new ProfiledPIDCommand(
//                this.controller,
//                this.slide::getDistance,
//                slide,
//                (output, state) -> {
//                    telemetry.addData("error", controller.getPositionError());
//                    telemetry.addData("output", output);
//                    this.slide.set(MathUtils.clamp(output, -0.5, 0.5));
//                },
//                this).alongWith(new InstantCommand(() -> {
//            this.flipper.setTargetDistance(flipperPosition.getPosition());
////            this.flipper.set(0.2);
//        })).interruptOn(this.flipper::atTargetPosition).whenFinished(() -> this.flipper.set(0));
//    }

    public void moveIntake(double position) {
        intake.setPosition(position);
    }

    public void closeClaw() {
        moveIntake(Constants.Lift.IntakePosition.CLOSED.getPosition());
    }

    public void openClaw() {
        moveIntake(Constants.Lift.IntakePosition.OPEN.getPosition());
    }

    public void setPosition(Constants.Lift.Position position) {
        setPosition(position.getSlideHeightCentimeters(), position.getFlipperPosition());
    }

    public void setPosition(double slide, Constants.Lift.FlipperPosition flipperPosition) {
        targetSlide = slide;
        this.slide.setTargetDistance(slide);
        this.flipperSetpoint = flipperPosition.getPosition();

        this.controller.setSetPoint(this.flipperSetpoint);

//        this.flipperPower = flipperPosition == Constants.LinearSlide.FlipperPosition.UP ? -0.25 : -0.05;

//        this.flipper.setTargetDistance(flipperPosition.getPosition());
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
        return this.controller.atSetPoint();
    }

    @Override
    public void periodic() {
        slide.set(0.15);
        flipper.set(-controller.calculate(flipper.encoder.getPosition()));

        telemetry.addData("Flipper Output", controller.calculate(flipper.encoder.getPosition()));
        telemetry.addData("Intake Position", this.intake.getPosition());
//        telemetry.update();
    }
}