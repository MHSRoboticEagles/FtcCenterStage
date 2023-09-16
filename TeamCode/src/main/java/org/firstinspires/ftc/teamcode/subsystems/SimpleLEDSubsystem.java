package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SimpleLEDSubsystem implements Subsystem {
    String deviceName;
    Telemetry telemetry;
    DigitalChannel ledRed;
    DigitalChannel ledGreen;
    String sensorName;

    public static boolean TELEMETRY_ENABLED = true;

    public SimpleLEDSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String deviceName, String redDeviceName, String greenDeviceName) {
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.ledRed = hardwareMap.get(DigitalChannel.class, redDeviceName);
        this.ledGreen = hardwareMap.get(DigitalChannel.class, greenDeviceName);
        this.sensorName = String.format("led:%s", deviceName);
        this.ledRed.setMode(DigitalChannel.Mode.OUTPUT);
        this.ledGreen.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setState00() {
        this.setLed(false, false, "00");
    }

    public void setState01() {
        this.setLed(false, true, "01");
    }

    public void setState02() {
        this.setLed(true, false, "10");
    }

    public void setState03() {
        this.setLed(true, true, "11");
    }

    private void setLed(boolean red, boolean green, String message) {
        ledRed.setState(red);
        ledGreen.setState(green);
        if (TELEMETRY_ENABLED) {
            telemetry.addData(sensorName, message);
        }
    }

    @Override
    public void periodic() {
        // do nothing at all
    }
}
