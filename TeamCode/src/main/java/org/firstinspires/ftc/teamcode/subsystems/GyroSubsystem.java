package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class GyroSubsystem implements Subsystem {
    Telemetry telemetry;
    IMU imu;

    public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    protected RevHubOrientationOnRobot orientationOnRobot;

    public static double INITIAL_HEADING_DEGREES = 0.0;
    public static boolean TELEMETRY_ENABLED = false;

    protected double initialHeading;

    public GyroSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        initialHeading = INITIAL_HEADING_DEGREES;
        resetHeading();
    }

    @Override
    public void periodic() {
        // do nothing
    }

    /**
     * Resets the heading to 0.0
     */
    public void resetHeading() {
        this.resetHeading(0.0);
    }

    /**
     * Resets the heading to a new value (in degrees)
     * @param newHeading new heading (in degrees)
     */
    public void resetHeading(double newHeading) {
        this.initialHeading = newHeading;
        imu.resetYaw();
    }

    /**
     * Gets the current heading value from the IMU
     * @return heading (in degrees)
     */
    public double getHeading() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        yaw += initialHeading;
        yaw %= 360.0;
        if (yaw > 180.0) {
            yaw -= 360.0;
        }
        return yaw;
    }

    /**
     * Gets the current heading value from the IMU
     * @return heading (in radians)
     */
    public double getHeadingRadians() {
        return Math.toRadians(getHeading());
    }

    /**
     * Gets the current angular velocity from the IMU
     * @return angular velocity (in degrees per second)
     */
    public double getAngularVelocity() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }

    /**
     * Gets the current angular velocity from the IMU
     * @return angular velocity (in radians per second)
     */
    public double getAngularVelocityRadians() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

}
