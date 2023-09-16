package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DrivetrainSubsystem implements Subsystem {

    private final MecanumDrive drive;

    private final Telemetry telemetry;

    public static double TRACKWIDTH = 8.354;
    public static double CENTER_WHEEL_OFFSET = -4.7;
    public static double WHEEL_DIAMETER = (35.0 / 25.4);
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private final Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;
    protected Pose2d robotPose;

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose) {
        this(hardwareMap, telemetry);
        this.setPose(initialPose);
    }

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Drive------------------

        MotorEx frontLeft, frontRight, backLeft, backRight;

        this.telemetry = telemetry;

        frontLeft = new MotorEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        backLeft = new MotorEx(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312);
        backRight = new MotorEx(hardwareMap, "backRight", Motor.GoBILDA.RPM_312);

        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);
        backRight.setInverted(true);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Odometry------------------

        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = backRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.setDirection(Motor.Direction.REVERSE);
        rightOdometer.setDirection(Motor.Direction.REVERSE);
        centerOdometer.setDirection(Motor.Direction.FORWARD);

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();
    }

    @Override
    public void periodic() {
        this.update(
                leftOdometer.getDistance(),
                rightOdometer.getDistance(),
                centerOdometer.getDistance()
        );
    }

    //Drive-----------------------------

    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro (degrees)
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle - 90);
    }

    /**
     * Stop the motors.
     */
    public void stop() {
        drive.stop();
    }


    /**
     * Drives the motors directly with the specified motor powers.
     *
     * @param frontLeftSpeed    the speed of the front left motor
     * @param frontRightSpeed   the speed of the front right motor
     * @param backLeftSpeed     the speed of the back left motor
     * @param backRightSpeed    the speed of the back right motor
     */
    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        drive.driveWithMotorPowers(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    //Odometry-----------------------------

    public void setPose(double xInches, double yInches, double headingDegrees) {
        this.setPose(new Pose2d(xInches, yInches, Rotation2d.fromDegrees(headingDegrees)));
    }

    public void setPose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {

        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaRightEncoder - deltaLeftEncoder) / TRACKWIDTH
                )
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (CENTER_WHEEL_OFFSET * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }

    public Pose2d getPose() {
        return robotPose;
    }
}
