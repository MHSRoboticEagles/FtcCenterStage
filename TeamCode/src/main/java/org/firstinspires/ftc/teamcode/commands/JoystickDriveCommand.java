package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.GyroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class JoystickDriveCommand extends CommandBase {

    public static boolean FIELD_CENTRIC_DRIVE = true;

    DrivetrainSubsystem driveTrain;
    GyroSubsystem gyro;
    private final DoubleSupplier xPowerSupplier;
    private final DoubleSupplier yPowerSupplier;
    private final DoubleSupplier turnPowerSupplier;

    private final Telemetry telemetry;

    public JoystickDriveCommand(Telemetry telemetry, DrivetrainSubsystem driveTrain, DoubleSupplier xPower, DoubleSupplier yPower, DoubleSupplier turnPower, GyroSubsystem gyro) {
        this.telemetry = telemetry;
        this.driveTrain = driveTrain;
        this.gyro = gyro;

        this.xPowerSupplier = xPower;
        this.yPowerSupplier = yPower;
        this.turnPowerSupplier = turnPower;

        addRequirements(driveTrain);
        addRequirements(gyro);
    }

    @Override
    public void initialize() {
        // do nothing
    }

    @Override
    public void execute() {
        if (FIELD_CENTRIC_DRIVE) {
            executeFieldCentric();
        } else {
            executeRobotCentric();
        }
    }

    protected void executeFieldCentric() {
        double xPower = xPowerSupplier.getAsDouble();
        double yPower = yPowerSupplier.getAsDouble();
        double turnPower = turnPowerSupplier.getAsDouble();

//        telemetry.addData("FCD.xPower", "%.3f", xPower);
//        telemetry.addData("FCD.yPower", "%.3f", yPower);
//        telemetry.addData("FCD.tPower", "%.3f", turnPower);
//        telemetry.addData("FCD.gyro", "%.3f", gyro.getHeading());

        driveTrain.driveFieldCentric(xPower, yPower, turnPower, gyro.getHeading());
    }

    protected void executeRobotCentric() {
        double xPower = xPowerSupplier.getAsDouble();
        double yPower = yPowerSupplier.getAsDouble();
        double turnPower = turnPowerSupplier.getAsDouble();

//        telemetry.addData("RCD.xPower", "%.3f", xPower);
//        telemetry.addData("RCD.yPower", "%.3f", yPower);
//        telemetry.addData("RCD.tPower", "%.3f", turnPower);

        driveTrain.driveRobotCentric(xPower, yPower, turnPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
