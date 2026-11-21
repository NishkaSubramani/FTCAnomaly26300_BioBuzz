package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Timer;

public abstract class BaseTeleOp extends LinearOpMode {

    protected Hardware robotHardware;
    protected Intake intake;
    protected GoBildaPinpointDriver pinpoint;
    protected GamepadEx controller1;

    protected TriggerReader lt1Reader;
    protected TriggerReader rt1Reader;

    private double outtakePower = 0.0;

    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousB = false;

    private double lastLoopEndMs = 0;

    private final double[] lastDrivePower = {
            Double.NaN, Double.NaN, Double.NaN, Double.NaN
    };

    private static final double POWER_TOLERANCE = 0.01;

    public void initHardware() {
        initHardware(true);
    }

    public void initHardware(boolean usePinpoint) {

        robotHardware = new Hardware(hardwareMap, usePinpoint);

        intake = robotHardware.robotIntake;
        pinpoint = robotHardware.robotPinpoint;

        controller1 = new GamepadEx(gamepad1);

        lt1Reader = new TriggerReader(
                controller1,
                GamepadKeys.Trigger.LEFT_TRIGGER
        );

        rt1Reader = new TriggerReader(
                controller1,
                GamepadKeys.Trigger.RIGHT_TRIGGER
        );
    }

    public void updateInputs() {

        controller1.readButtons();
        lt1Reader.readValue();
        rt1Reader.readValue();

        // No Pinpoint update in RobotCentric
    }

    // Intake and indexer motor together
    protected void updateIntake() {

        if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {

            intake.out();
            robotHardware.indexerMotor.setPower(-1.0);

        } else if (gamepad1.left_trigger > 0.05) {

            intake.in();
            robotHardware.indexerMotor.setPower(1.0);

        } else {

            intake.stop();
            robotHardware.indexerMotor.setPower(0.0);
        }
    }

    // X = 0.4, Y = 0.5, B = 0.6
    // Press same button again to stop
    protected void updateOuttake() {

        boolean xPressed =
                controller1.getButton(GamepadKeys.Button.X);

        boolean yPressed =
                controller1.getButton(GamepadKeys.Button.Y);

        boolean bPressed =
                controller1.getButton(GamepadKeys.Button.B);

        if (xPressed && !previousX) {
            outtakePower = (outtakePower == 0.4) ? 0.0 : 0.4;
        }

        if (yPressed && !previousY) {
            outtakePower = (outtakePower == 0.5) ? 0.0 : 0.5;
        }

        if (bPressed && !previousB) {
            outtakePower = (outtakePower == 0.6) ? 0.0 : 0.6;
        }

        previousX = xPressed;
        previousY = yPressed;
        previousB = bPressed;

        robotHardware.outtakeMotor.setPower(outtakePower);
    }

    // Right bumper controls both continuous servos
    protected void updateIndexer() {

        if (controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {

            robotHardware.indexerServo1.setPower(0.5);
            robotHardware.indexerServo2.setPower(0.5);

        } else {

            robotHardware.indexerServo1.setPower(0.0);
            robotHardware.indexerServo2.setPower(0.0);
        }
    }

    protected void updateSubsystems() {

        updateIntake();
        updateOuttake();
        updateIndexer();
    }

    protected void setDrivePowers(
            double fl, double fr, double bl, double br
    ) {

        setIfChanged(robotHardware.frontLeft, 0, fl);
        setIfChanged(robotHardware.frontRight, 1, fr);
        setIfChanged(robotHardware.backLeft, 2, bl);
        setIfChanged(robotHardware.backRight, 3, br);
    }

    private void setIfChanged(
            DcMotorEx motor, int index, double power
    ) {

        double last = lastDrivePower[index];

        boolean firstTime = Double.isNaN(last);
        boolean stopping = power == 0 && last != 0;

        boolean bigChange =
                Math.abs(power - last) >= POWER_TOLERANCE;

        if (firstTime || stopping || bigChange) {

            motor.setPower(power);
            lastDrivePower[index] = power;
        }
    }

    protected void addLoopTimeTelemetry() {

        double now = Timer.milliseconds();

        telemetry.addData(
                "Loop (ms)",
                "%.1f",
                now - lastLoopEndMs
        );

        lastLoopEndMs = now;
    }

    protected void stopAllMotors() {

        setDrivePowers(0, 0, 0, 0);

        intake.stop();

        robotHardware.indexerMotor.setPower(0.0);
        robotHardware.outtakeMotor.setPower(0.0);

        robotHardware.indexerServo1.setPower(0.0);
        robotHardware.indexerServo2.setPower(0.0);
    }
}