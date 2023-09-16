package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Waypoint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroSubsystem;

public class PurePursuitCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private GyroSubsystem gyro;
    private Path m_path;

    private static String TAG = "PurePursuitCommand";

    private Telemetry telemetry;

    public PurePursuitCommand(DrivetrainSubsystem drivetrain, GyroSubsystem gyro, Telemetry telemetry, Waypoint... waypoints) {
        m_path = new Path(waypoints);
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;
        this.gyro = gyro;
        Log.d(TAG, "PurePursuitCommand: " + waypoints.length + " waypoints");
    }

    @Override
    public void initialize() {
        m_path.init();
    }

    public void addWaypoint(Waypoint waypoint) {
        m_path.add(waypoint);
    }

    public void addWaypoints(Waypoint... waypoints) {
        for (Waypoint waypoint : waypoints) this.addWaypoint(waypoint);
    }

    public void removeWaypointAtIndex(int index) {
        m_path.remove(index);
    }

    public Path getPath() {
        return m_path;
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        drivetrain.periodic();
        gyro.periodic();
        Pose2d robotPose = drivetrain.getPose();

        double xPos = robotPose.getTranslation().getX();
        double yPos = robotPose.getTranslation().getY();
        double heading = Math.toDegrees(robotPose.getHeading());

        telemetry.addData("ppc.segment", "%d of %d",  m_path.getCurrentWaypoint(), m_path.size()-1);
        telemetry.addData("ppc.loop", "%d",  m_path.loopCounter);

        telemetry.addData("ppc.tgt.x", "%.3f",  m_path.targetX);
        telemetry.addData("ppc.tgt.y", "%.3f", m_path.targetY);
        telemetry.addData("ppc.tgt.a", "%.3f", m_path.targetHeading);

        telemetry.addData("ppc.pos.x", "%.3f", xPos);
        telemetry.addData("ppc.pos.y", "%.3f", yPos);
        telemetry.addData("ppc.pos.a", "%.3f", heading);

        double[] motorSpeeds = m_path.newLoop(xPos, yPos, heading);

        telemetry.addData("ppc.power.x", "%.3f", motorSpeeds[0]);
        telemetry.addData("ppc.power.y", "%.3f", motorSpeeds[1]);
        telemetry.addData("ppc.power.a", "%.3f", Math.toDegrees(motorSpeeds[2]));

        if (motorSpeeds[0] == 0 && motorSpeeds[1] == 0 && motorSpeeds[2] == 0) {
            drivetrain.stop();
            return;
        }
        drivetrain.driveFieldCentric(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], gyro.getHeading());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return m_path.checkIfPathFinished();
    }
}
