package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Path;

import java.util.List;

/**
 * Set of helper functions for drawing Robot and  paths on dashboard canvases.
 */
public class DashboardUtil {
    private static final double ROBOT_RADIUS = 9; // in
    private static final String ROBOT_COLOR = "#740800";
    private static final String TARGET_PATH_COLOR = "#006c74";
    private static final String ROBOT_PATH_COLOR = "#b5683f";

    public static Telemetry dashboardTelemetry(Telemetry telemetry) {
        return dashboardTelemetry(telemetry, 100);
    }

    public static Telemetry dashboardTelemetry(Telemetry telemetry, int msTransmissionInterval) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(msTransmissionInterval);
        return new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        canvas.setStroke(ROBOT_PATH_COLOR);
        canvas.setFill(ROBOT_PATH_COLOR);
        canvas.setStrokeWidth(1);

        for (int i = 0; i < poseHistory.size(); i++) {
            xPoints[i] = poseHistory.get(i).getX();
            yPoints[i] = poseHistory.get(i).getY();
//            canvas.fillCircle(xPoints[i], yPoints[i], 2);
        }

        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        double x, y, a, x1, y1, x2, y2;
        x = pose.getX();
        y = pose.getY();
        a = pose.getHeading();

        x1 = x + (ROBOT_RADIUS / 2 * Math.cos(a));
        x2 = x + (ROBOT_RADIUS * Math.cos(a));
        y1 = y + (ROBOT_RADIUS / 2 * Math.sin(a));
        y2 = y + (ROBOT_RADIUS * Math.sin(a));

        canvas.setStroke(ROBOT_COLOR);
        canvas.setFill(ROBOT_COLOR);
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(x, y, ROBOT_RADIUS);
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawPath(Canvas canvas, Path path) {
        canvas.setStroke(TARGET_PATH_COLOR);
        canvas.setFill(TARGET_PATH_COLOR);
        canvas.setStrokeWidth(1);

        double[] xPoints = new double[path.size()];
        double[] yPoints = new double[path.size()];

        for (int i = 0; i < path.size(); i++) {
            xPoints[i] = path.get(i).getPose().getX();
            yPoints[i] = path.get(i).getPose().getY();
            canvas.fillCircle(xPoints[i], yPoints[i], 2);
        }

        canvas.strokePolyline(xPoints, yPoints);
    }
}
