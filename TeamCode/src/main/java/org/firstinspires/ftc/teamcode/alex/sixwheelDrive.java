package org.firstinspires.ftc.teamcode.alex;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
@Disabled

public class sixwheelDrive extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor frontleft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode(){
        imu=hardwareMap.get(Gyroscope.class, "imu");
        frontleft=hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight=hardwareMap.get(DcMotor.class, "frontRight");
        backLeft=hardwareMap.get(DcMotor.class, "backLeft");
        backRight=hardwareMap.get(DcMotor.class, "backRight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double rightStickInput;
        double leftStickInput;

        while (opModeIsActive()){

            rightStickInput=gamepad1.right_stick_y;
            leftStickInput=gamepad1.left_stick_y;
            frontleft.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(-1*gamepad1.right_stick_y);
            backRight.setPower(-1*gamepad1.right_stick_y);

        }

    }
}
