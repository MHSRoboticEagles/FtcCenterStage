package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroSubsystem;

@Config
public class MoveToPosCommand extends CommandBase {

    public static double TARGET_X = 0;
    public static double TARGET_Y = 0;
    public static double TARGET_A = 0;

    public static double DRIVE_SPEED = 0.3;

    public static double DIST_THRESHOLD = 0.5;
    public static double ANGLE_THRESHOLD = 3;

    private double actualDriveSpeed = 0.0;

    private DrivetrainSubsystem driveTrain;

    private GyroSubsystem gyro;
    private Telemetry telemetry;

        public MoveToPosCommand(DrivetrainSubsystem driveTrain, GyroSubsystem gyro, Telemetry telemetry) {
            this.driveTrain = driveTrain;
            this.gyro = gyro;
            this.telemetry = telemetry;
            addRequirements(driveTrain);
            addRequirements(gyro);
        }

        @Override
        public void initialize() {
            // do nothing
            actualDriveSpeed = DRIVE_SPEED;
        }

        double normalizedHeading(double heading) {
            double normalized = heading %= 360.0;
            if (normalized > 180.0) {
                normalized -= 360.0;
            }
            return normalized;
        }

        @Override
        public void execute() {

            Pose2d robotPose = driveTrain.getPose();

            double x_Distance = TARGET_X - robotPose.getX();
            double y_Distance = TARGET_Y - robotPose.getY();

            double turnPower = 0;

            double angle_Distance = this.normalizedHeading(gyro.getHeading() - TARGET_A);
            if (Math.abs(angle_Distance) > ANGLE_THRESHOLD) {
                turnPower = actualDriveSpeed * Math.signum(angle_Distance) * 0.5;
            }

            telemetry.addData("GTP.DiffX", "%.3f", x_Distance);
            telemetry.addData("GTP.DiffY", "%.3f", y_Distance);
            telemetry.addData("GTP.DiffA", "%.3f", angle_Distance);

            // normalize the distatnces
            double distance = Math.sqrt(Math.pow(x_Distance, 2) + Math.pow(y_Distance, 2));
            double xPower = (x_Distance / distance) * actualDriveSpeed;
            double yPower = (y_Distance / distance) * actualDriveSpeed;

            telemetry.addData("GTP.xPower", "%.3f", xPower);
            telemetry.addData("GTP.yPower", "%.3f", yPower);
            telemetry.addData("GTP.tPower", "%.3f", turnPower);
            telemetry.addData("GTP.gyro", "%.3f", gyro.getHeading());
            telemetry.addData("GTP.actualDriveSpeed", "%.3f", actualDriveSpeed);

            driveTrain.driveFieldCentric(xPower, yPower, turnPower, gyro.getHeading());
        }

        @Override
        public boolean isFinished() {
            double x_Distance = TARGET_X - driveTrain.getPose().getX();
            double y_Distance = TARGET_Y - driveTrain.getPose().getY();

            double distance = Math.sqrt(Math.pow(x_Distance, 2) + Math.pow(y_Distance, 2));

            double angle_Distance = Math.abs(   this.normalizedHeading(TARGET_A - gyro.getHeading()));

            if (distance < DIST_THRESHOLD && angle_Distance < ANGLE_THRESHOLD) {
//                return true;
                actualDriveSpeed = 0;
            } else {
                actualDriveSpeed = DRIVE_SPEED;
            }

            return false;
        }
}
