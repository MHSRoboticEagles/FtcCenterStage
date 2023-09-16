package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DistanceSensorSubsystem implements Subsystem {

    private final SensorRevTOFDistance distanceSensor;
    Telemetry telemetry;
    String deviceName;

    String sensorName;

    public static boolean TELEMETRY_ENABLED = false;
    public DistanceSensorSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
        this.deviceName = deviceName;
        this.telemetry = telemetry;
        distanceSensor = new SensorRevTOFDistance(hardwareMap, deviceName);
        this.sensorName = String.format("distance:%s", deviceName);
    }

    /**
     * Gets the distance detected by the proximity sensor.
     * @return Double representing the distance in inches
     */
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    /**
     * Gets the distance detected by the proximity sensor.
     * @return Double representing the distance in CM
     */
    public double getDistanceCM() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    @Override
    public void periodic() {
        if (TELEMETRY_ENABLED) {
            telemetry.addData(sensorName, "%.2f", getDistance());
        }
    }
}
