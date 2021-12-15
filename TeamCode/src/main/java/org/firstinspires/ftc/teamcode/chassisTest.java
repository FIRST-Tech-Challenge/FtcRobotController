//
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class chassisTest extends LinearOpMode {

    //Motor initalization
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor carousel;
    private DcMotor crane;
    private Servo arm;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        carousel = hardwareMap.get(DcMotor.class, "carousel");
        crane = hardwareMap.get(DcMotor.class, "crane");
        arm = hardwareMap.get(Servo.class, "arm");

        //setting motor direction since some motors were backwards
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

            double elevatorPower;
            double carouselPower;
            double armPower = 0;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeValue = gamepad1.left_stick_x;
            elevatorPower = gamepad2.left_stick_y;
            armPower = gamepad2.right_stick_x;
            carouselPower = gamepad2.left_trigger;

            //making motors run.
            //strafing
            if (strafeValue > 0.1) {
                frontLeft.setPower(-strafeValue);
                frontRight.setPower(strafeValue);
                backLeft.setPower(strafeValue);
                backRight.setPower(-strafeValue);
            } else if (strafeValue < -0.1) {
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

            crane.setPower(elevatorPower);

            arm.setPosition(armPower);
//            if(armPower > 0){
//                arm.setDirection(Servo.Direction.FORWARD);
//            }else{
//                arm.setDirection(Servo.Direction.REVERSE);
//            }

            carousel.setPower(carouselPower);

            telemetry.addData("Arm power", armPower);
            telemetry.addData("Arm position", arm.getPosition());
            telemetry.update();
        }
        //telemetry for debugging


    }
}
