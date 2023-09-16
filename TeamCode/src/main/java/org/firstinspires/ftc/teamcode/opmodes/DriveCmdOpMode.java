package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.JoystickDriveCommand;
import org.firstinspires.ftc.teamcode.commands.MoveToPosCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GyroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleLEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TouchSensorSubsystem;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name = "0-DriveCmd", group = "drive")
//@Disabled
@Config
public class DriveCmdOpMode extends CommandOpMode {

    public static double START_A = 0.00;
    public static double START_X = 0.00;
    public static double START_Y = 0.00;

    GyroSubsystem gyro;
    DrivetrainSubsystem driveSubsystem;

    DistanceSensorSubsystem leftDistanceSensor;
    DistanceSensorSubsystem rightDistanceSensor;
    DistanceSensorSubsystem frontDistanceSensor;
    ColorSensorSubsystem colorSensor;
    TouchSensorSubsystem touchSensorOne;
    SimpleLEDSubsystem led1;

    GamepadEx m_driverOp;
    JoystickDriveCommand joystickDriveCommandCommand;
    MoveToPosCommand moveToPosCommand;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        setupGamepads();
        setupSubsystems();
        setupCommands();
        setupButtons();
        setupDefaultCommands();

        schedule(new RunCommand(() -> {
            driveSubsystem.periodic();
            Pose2d pose = driveSubsystem.getPose(); // get the updated position
            double xPos = pose.getX();
            double yPos = pose.getY();
            double heading = pose.getHeading();
            telemetry.addData("x", "%.3f",  xPos);
            telemetry.addData("y", "%.3f", yPos);
            telemetry.addData("a", "%.3f", Math.toDegrees(heading));
            telemetry.addData("Gyro.A", "%.3f", gyro.getHeading());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(packet.fieldOverlay(),pose);
            dashboard.sendTelemetryPacket(packet);
        }));
    }

    private void setupGamepads() {
        m_driverOp = new GamepadEx(gamepad1);
    }

    protected void setupSubsystems() {

        gyro = new GyroSubsystem(hardwareMap, telemetry);
        gyro.resetHeading(START_A);
        register(gyro);

        driveSubsystem = new DrivetrainSubsystem(hardwareMap, telemetry);
        driveSubsystem.setPose(START_X, START_Y, START_A);
        register(driveSubsystem);

        leftDistanceSensor = new DistanceSensorSubsystem(hardwareMap, telemetry, "leftDistance");
        rightDistanceSensor = new DistanceSensorSubsystem(hardwareMap, telemetry, "rightDistance");
        frontDistanceSensor = new DistanceSensorSubsystem(hardwareMap, telemetry, "frontDistance");
        register(leftDistanceSensor);
        register(rightDistanceSensor);
        register(frontDistanceSensor);

        led1 = new SimpleLEDSubsystem(hardwareMap, telemetry, "led1", "red1", "green1");
        register(led1);

        colorSensor = new ColorSensorSubsystem(hardwareMap, telemetry, "backColor");
        register(colorSensor);

        touchSensorOne = new TouchSensorSubsystem(hardwareMap, telemetry, "touchOne");
        register(touchSensorOne);
    }

    protected void setupCommands() {
        joystickDriveCommandCommand = new JoystickDriveCommand(
                telemetry,
                driveSubsystem,
                m_driverOp::getLeftX,
                m_driverOp::getLeftY,
                m_driverOp::getRightX,
                gyro);

        moveToPosCommand = new MoveToPosCommand(driveSubsystem, gyro, telemetry);
    }

    private void setupButtons() {

        m_driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            moveToPosCommand.schedule();
        });
    }

    private void setupDefaultCommands() {
        driveSubsystem.setDefaultCommand(joystickDriveCommandCommand);
    }
}
