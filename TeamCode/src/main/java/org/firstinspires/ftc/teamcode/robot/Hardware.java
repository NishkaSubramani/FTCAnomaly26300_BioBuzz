package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class Hardware {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx intakeMotor, indexerMotor, outtakeMotor;

    public CRServo indexerServo1, indexerServo2;

    public Intake robotIntake;
    public GoBildaPinpointDriver robotPinpoint;

    public Hardware(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public Hardware(HardwareMap hardwareMap, boolean usePinpoint) {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        indexerMotor = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        indexerServo1 = hardwareMap.get(CRServo.class, "indexerServo1");
        indexerServo2 = hardwareMap.get(CRServo.class, "indexerServo2");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        indexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Original motor directions preserved
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        indexerMotor.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        robotIntake = new Intake(intakeMotor);

        // Only initialize Pinpoint when the OpMode needs it
        if (usePinpoint) {

            robotPinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class,
                    "pinpoint"
            );

            robotPinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
            );

            robotPinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            robotPinpoint.setOffsets(0, 0, DistanceUnit.MM);
            robotPinpoint.resetPosAndIMU();

        } else {
            robotPinpoint = null;
        }

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}