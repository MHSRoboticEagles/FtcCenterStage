package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TouchSensorSubsystem implements Subsystem {
    TouchSensor touchSensor;
    String deviceName;
    Telemetry telemetry;

    String sensorName;

    public static boolean TELEMETRY_ENABLED = false;

    public TouchSensorSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.touchSensor = hardwareMap.get(TouchSensor.class, deviceName);
        this.sensorName = String.format("touch:%s", deviceName);
    }

    public boolean isPressed() {
        return touchSensor.isPressed();
    }

    public boolean isNotPressed() {
        return !isPressed();
    }

    @Override
    public void periodic() {
        if (TELEMETRY_ENABLED) {
            telemetry.addData(sensorName, "%b", isPressed());
        }
    }
}
