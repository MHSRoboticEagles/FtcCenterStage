package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroSubsystem;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

@Autonomous
//@Disabled
public class PurePursuitSample extends CommandOpMode {

    private DrivetrainSubsystem drivetrain;
    private GyroSubsystem gyro;
    private PurePursuitCommand ppCommand;

    List<Pose2d> poseHistory = new ArrayList<>();

    @Override
    public void initialize() {

        telemetry = DashboardUtil.dashboardTelemetry(telemetry);

        // create our drive object
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        gyro = new GyroSubsystem(hardwareMap, telemetry);

        drivetrain.setPose(0, 0, 0);
        gyro.resetHeading(0);

        // create our pure pursuit command
        ppCommand = this.constructPath();

        showOnDashboard();

        // schedule the command
        schedule(ppCommand);

        schedule(new RunCommand(this::showOnDashboard).withTimeout(30 * 1000));
    }

    private void showOnDashboard() {
        telemetry.update();

        drivetrain.periodic();
        Pose2d pose = drivetrain.getPose(); // get the updated position

        if (poseHistory.size() > 0) {
            Pose2d lastPose = poseHistory.get(poseHistory.size() - 1);
            double distance = lastPose.getTranslation().getDistance(pose.getTranslation());
            if (distance > 0.25)
                poseHistory.add(pose);
        } else {
            poseHistory.add(pose);
        }

        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawPath(packet.fieldOverlay(),ppCommand.getPath());
        DashboardUtil.drawPoseHistory(packet.fieldOverlay(),poseHistory);
        DashboardUtil.drawRobot(packet.fieldOverlay(),pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


    private PurePursuitCommand constructPath() {
        PurePursuitCommand cmd = new PurePursuitCommand(drivetrain, gyro, telemetry);

        cmd.addWaypoint(new StartWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
        cmd.addWaypoint(new PointTurnWaypoint(48, 0, Math.toRadians(0), 0.4, 0.2, 8, 7, 5));
        cmd.addWaypoint(new PointTurnWaypoint(48, 24, Math.toRadians(0), 0.5, 0.45, 8, 7, 4));
        cmd.addWaypoint(new PointTurnWaypoint(36, 24, Math.toRadians(45), 0.5, 0.5, 8, 2, 10));
        cmd.addWaypoint(new PointTurnWaypoint(24, 24, Math.toRadians(90), 0.5, 0.3, 8, 1.5, 4));
        cmd.addWaypoint(new PointTurnWaypoint(68, 24, Math.toRadians(-180), 0.4, 0.3, 8, 2, 2));
        cmd.addWaypoint(new EndWaypoint(72, 24, Math.toRadians(-180), 0.15, 0.15, 8, 0.5, 1));
//        cmd.addWaypoint(new EndWaypoint(0,0,Math.toRadians(0),0.3,0.2,0,1,2));

        return cmd;
    }
}