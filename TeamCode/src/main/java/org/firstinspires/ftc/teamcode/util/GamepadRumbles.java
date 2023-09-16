package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadRumbles {
    public static Gamepad.RumbleEffect threeBeeps = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 250)
            .addStep(0.0, 0.0, 250)
            .addStep(1.0, 1.0, 250)
            .addStep(0.0, 0.0, 250)
            .addStep(1.0, 1.0, 250)
            .build();
}
