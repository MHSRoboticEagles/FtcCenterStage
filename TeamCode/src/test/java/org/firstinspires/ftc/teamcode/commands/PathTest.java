package org.firstinspires.ftc.teamcode.commands;

import static org.junit.jupiter.api.Assertions.*;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;

import org.junit.jupiter.api.Test;

class PathTest {
    @Test
    void testFirstStep() {
        Path path = new Path();
        path.add(new StartWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
        path.add(new GeneralWaypoint(10, 0, 0.3, 0.3, 8));
        path.add(new EndWaypoint(
                20, 20, 0, 0.4,
                0.3, 10, 0.8, 1
        ));
        path.init();
        double[] speeds = path.loop(1, 0, 0);
        assertEquals(1.0, speeds[0]);
        assertEquals(0.0, speeds[1]);
        assertEquals(0.0, speeds[2]);
    }
}