package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Drivetrain extends SubsystemBase {
    private final MotorEx frontLeft;
    private final MotorEx frontRight;
    private final MotorEx backLeft;
    private final MotorEx backRight;

    private final IMU gyro;

    private final ElapsedTime timer;

    private final MecanumDrive drive;

    private final MecanumDriveOdometry odometry;
    private final MecanumDriveKinematics kinematics;

    private DriveMode mode = DriveMode.NORMAL;

    private Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.frontLeft = new MotorEx(hardwareMap, "FrontLeft", Motor.GoBILDA.RPM_312);
        this.frontRight = new MotorEx(hardwareMap, "FrontRight", Motor.GoBILDA.RPM_312);
        this.backLeft = new MotorEx(hardwareMap, "BackLeft", Motor.GoBILDA.RPM_312);
        this.backRight = new MotorEx(hardwareMap, "BackRight", Motor.GoBILDA.RPM_312);

        this.telemetry = telemetry;

        // TODO: ensure this works
        this.gyro = hardwareMap.get(IMU.class, "imu");

        gyro.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT // TODO: confirm
                )
            )
        );

        this.frontLeft.setDistancePerPulse(Constants.Drivetrain.metersPerTick);
        this.frontRight.setDistancePerPulse(Constants.Drivetrain.metersPerTick);
        this.backLeft.setDistancePerPulse(Constants.Drivetrain.metersPerTick);
        this.backRight.setDistancePerPulse(Constants.Drivetrain.metersPerTick);

        this.drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.kinematics = new MecanumDriveKinematics(
                Constants.Drivetrain.frontLeftWheelMeters,
                Constants.Drivetrain.frontRightWheelMeters,
                Constants.Drivetrain.backLeftWheelMeters,
                Constants.Drivetrain.backRightWheelMeters
        );

        this.odometry = new MecanumDriveOdometry(kinematics, new Rotation2d(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        this.timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    @Override
    public void periodic() {
        MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds(
                this.frontLeft.getRate(),
                this.frontRight.getRate(),
                this.backLeft.getRate(),
                this.backRight.getRate()
        );
        Rotation2d heading = Rotation2d.fromDegrees(getHeadingDegrees());

        telemetry.addData("rotation", getHeadingDegrees());
        telemetry.addData("forward", odometry.getPoseMeters().getX());
        telemetry.addData("sideways", odometry.getPoseMeters().getY());
        telemetry.addData("mode", this.mode == DriveMode.NORMAL ? "normal" : "slow");

        this.odometry.updateWithTime(this.timer.time(), heading, speeds);

    }

    public double getHeading() {
        return this.gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return this.gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetOdometry(Pose2d pose) {
        this.odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeadingDegrees()));
    }

    public Command teleopDrive(GamepadEx gamepad) {
        return new RunCommand(
            () -> this.drive.driveRobotCentric(
                    -gamepad.getLeftX() * mode.getMultiplier(),
                    -gamepad.getLeftY() * mode.getMultiplier(),
                    -gamepad.getRightX() * mode.getMultiplier(),
                    false),
            this
        );
    }

    public void setMode(DriveMode mode) {
        this.mode = mode;
    }

    public MotorEx getFrontLeft() {
        return frontLeft;
    }

    public MotorEx getFrontRight() {
        return frontRight;
    }

    public MotorEx getBackLeft() {
        return backLeft;
    }

    public MotorEx getBackRight() {
        return backRight;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public IMU getGyro() {
        return gyro;
    }

    public void setPower(double flPower, double frPower, double blPower, double brPower){
        frontLeft.set(flPower);
        frontRight.set(frPower);
        backLeft.set(blPower);
        backRight.set(brPower);
    }

    public void setTargetPose(double targetPose){
        frontLeft.setTargetDistance(targetPose * Constants.Drivetrain.ticksPerMeter);
        frontRight.setTargetDistance(targetPose* Constants.Drivetrain.ticksPerMeter);
        backLeft.setTargetDistance(targetPose* Constants.Drivetrain.ticksPerMeter);
        backRight.setTargetDistance(targetPose* Constants.Drivetrain.ticksPerMeter);
    }

    public void setKP(double kp){
        frontLeft.setPositionCoefficient(kp);
        frontRight.setPositionCoefficient(kp);
        backLeft.setPositionCoefficient(kp);
        backRight.setPositionCoefficient(kp);
    }

    public enum DriveMode {
        NORMAL(0.75),
        SLOW(0.321);

        private final double multiplier;
        DriveMode(double multiplier) {
            this.multiplier = multiplier;
        }

        public double getMultiplier() {
            return multiplier;
        }
    }

    public void setRunMode(Motor.RunMode mode) {
        frontLeft.setRunMode(mode);
        frontRight.setRunMode(mode);
        backLeft.setRunMode(mode);
        backRight.setRunMode(mode);
    }
}