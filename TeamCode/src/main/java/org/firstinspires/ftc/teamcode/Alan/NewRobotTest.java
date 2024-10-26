package org.firstinspires.ftc.teamcode.Alan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//test drive for new INTO THE DEEP robot
@TeleOp
public class NewRobotTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //getting motors
        DcMotor leftFront = hardwareMap.dcMotor.get("fLeft");
        DcMotor rightFront = hardwareMap.dcMotor.get("fRight");
        DcMotor leftBack = hardwareMap.dcMotor.get("bLeft");
        DcMotor rightBack = hardwareMap.dcMotor.get("bRight");

        //setting motors to the right rotation direction
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeInInit()) {
            telemetry.addData("W", "");
            telemetry.update();
        }
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            leftFront.setPower(forward + turn + strafe);
            rightFront.setPower(forward - turn - strafe);
            leftBack.setPower(forward + turn - strafe);
            rightBack.setPower(forward - turn + strafe);
        }
    }
}
