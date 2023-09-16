package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Point;

import java.util.ArrayList;

public class Geometry {
    public static double angleWrap(double headingDegrees) {
        double normalized = headingDegrees % 360.0;
        if (normalized > 180.0) {
            normalized -= 360.0;
        } else if (normalized < -180.0) {
            normalized += 360.0;
        }
        return normalized;
    }

    public static double angleWrapRadians(double headingRadians) {
        double normalized = headingRadians % (2 * Math.PI);
        if (normalized > Math.PI) {
            normalized -= (2 * Math.PI);
        } else if (normalized < Math.PI) {
            normalized += (2 * Math.PI);
        }
        return normalized;
    }
}
