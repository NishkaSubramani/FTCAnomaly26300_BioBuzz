
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Timer;

@TeleOp(name = "RobotCentric TeleOp", group = "TeleOp")
public class RobotCentric extends BaseTeleOp {

    // Temporary speed limit for drivetrain testing
    private static final double DRIVE_SPEED = 0.75;
    private static final double DEADZONE = 0.05;

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Disable Pinpoint for robot-centric driving
        initHardware(false);

        telemetry.addData("Status", "INITIALIZED");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        Timer.start();

        try {
            while (opModeIsActive()) {

                updateInputs();

                // JOYSTICK INPUTS
                double ly = applyDeadzone(controller1.getLeftY());
                double lx = applyDeadzone(controller1.getLeftX());
                double rx = applyDeadzone(controller1.getRightX());

                // MECANUM DRIVE CALCULATIONS
                double frontLeftPower = ly + lx + rx;
                double frontRightPower = ly - lx - rx;
                double backLeftPower = ly - lx + rx;
                double backRightPower = ly + lx - rx;

                // NORMALIZE MOTOR POWERS
                double maxPower = Math.max(
                        1.0,
                        Math.max(
                                Math.max(
                                        Math.abs(frontLeftPower),
                                        Math.abs(frontRightPower)
                                ),
                                Math.max(
                                        Math.abs(backLeftPower),
                                        Math.abs(backRightPower)
                                )
                        )
                );

                // APPLY TEMPORARY SPEED LIMIT
                setDrivePowers(
                        (frontLeftPower / maxPower) * DRIVE_SPEED,
                        (frontRightPower / maxPower) * DRIVE_SPEED,
                        (backLeftPower / maxPower) * DRIVE_SPEED,
                        (backRightPower / maxPower) * DRIVE_SPEED
                );

                // INTAKE, INDEXER, OUTTAKE, SERVOS
                updateSubsystems();

                // TELEMETRY
                telemetry.addData("Status", "RUNNING");
                telemetry.addData("Left Y", "%.2f", ly);
                telemetry.addData("Left X", "%.2f", lx);
                telemetry.addData("Right X", "%.2f", rx);
                telemetry.addData("Drive Speed", "%.0f%%",
                        DRIVE_SPEED * 100);

                addLoopTimeTelemetry();
                telemetry.update();
            }

        } finally {
            stopAllMotors();
        }
    }
}
