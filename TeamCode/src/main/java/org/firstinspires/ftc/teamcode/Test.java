package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
public class Test extends LinearOpMode {
    private DcMotor
            frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive,
            armMotor;
    private final int BOTTOM = 0;
    private final int MIDDLE = 250;
    private final int TOP = 500;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.dcMotor.get("FL");
        frontRightDrive = hardwareMap.dcMotor.get("FR");
        backLeftDrive = hardwareMap.dcMotor.get("BL");
        backRightDrive = hardwareMap.dcMotor.get("BR");

//        armMotor = hardwareMap.dcMotor.get("armMotor");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Values can be customized
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

            frontLeftDrive.setPower(-gamepad1.left_stick_y);
            frontRightDrive.setPower(-gamepad1.right_stick_y);
            backLeftDrive.setPower(gamepad1.left_stick_x);
            backRightDrive.setPower(gamepad1.right_stick_x);

            telemetry.addData("Front Left Power", frontLeftDrive.getPower());
            telemetry.addData("Front Right Power", frontRightDrive.getPower());
            telemetry.addData("Back Left Power", backLeftDrive.getPower());
            telemetry.addData("Back Right Power", backRightDrive.getPower());
            telemetry.update();
        }
    }
}
