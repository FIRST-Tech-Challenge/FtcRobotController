//
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class chassisTest extends LinearOpMode {

    //Motor initalization
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            //defining driving variables.
            double throttle;
            double turn;
            double strafeValue;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeValue = gamepad1.left_stick_x;

            //making motors run.
                //strafing
            if (strafeValue > 0.1){
                frontLeft.setPower(-strafeValue);
                frontRight.setPower(strafeValue);
                backLeft.setPower(strafeValue);
                backRight.setPower(-strafeValue);
            }else if (strafeValue < -0.1){
                frontLeft.setPower(-strafeValue);
                frontRight.setPower(strafeValue);
                backLeft.setPower(strafeValue);
                backRight.setPower(-strafeValue);
            }
            //forward and backward movement
            frontLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backLeft.setPower(throttle);
            backRight.setPower(throttle);

            //turning
            frontLeft.setPower(-turn);
            frontRight.setPower(turn);
            backLeft.setPower(-turn);
            backRight.setPower(turn);
            }
//            telemetry.addData("Throttle", throttle);
//            telemetry.addData("Strafing", strafeValue);
//            telemetry.addData("Turning", turn);
//            telemetry.update();
    }
}
