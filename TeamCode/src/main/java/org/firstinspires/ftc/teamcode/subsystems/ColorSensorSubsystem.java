package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ColorSensorSubsystem implements Subsystem {

    SensorColor colorSensor;
    Telemetry telemetry;
    String deviceName;

    String sensorName;

    public static boolean TELEMETRY_ENABLED = false;

    public ColorSensorSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
        colorSensor = new SensorColor(hardwareMap, deviceName);
        this.deviceName = deviceName;
        this.telemetry = telemetry;
        this.sensorName = String.format("color:%s", deviceName);
    }

    @Override
    public void periodic() {
        if (TELEMETRY_ENABLED) {
            telemetry.addData(deviceName, "RGBA: %d,%d,%d,%d", colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha());
        }
    }
}
