package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;

@TeleOp(name = "iWearSheistiesToAvoidTheOps")

public class iWearSheistiesToAvoidTheOps extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor LFMotor = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor LBMotor = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor RFMotor = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor RBMotor = hardwareMap.dcMotor.get("motorBackRight");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested())return;

        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("turn", turn);

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double leftFront = power * cos/max + turn;
            double rightFront = power * cos/max - turn;
            double leftBack = power * cos/max + turn;
            double rightBack = power * cos/max - turn;



            if((power + Math.abs(turn)) > 1){
                leftFront /= power + turn;
                rightFront /= power + turn;
                leftBack /= power + turn;
                rightBack /= power + turn;
            }

            telemetry.addData("leftFront", leftFront);
            telemetry.addData("rightFront", rightFront);
            telemetry.addData("leftBack", leftBack);
            telemetry.addData("rightBack", rightBack);

            LFMotor.setPower(leftFront);
            RFMotor.setPower(rightFront);
            LBMotor.setPower(leftBack);
            RBMotor.setPower(rightBack);
 /*
            RFMotor.setPower(pivot + (-vertical + horizontal));
            RBMotor.setPower(pivot + (-vertical - horizontal));
            LFMotor.setPower(pivot + (-vertical - horizontal));
            LBMotor.setPower(pivot + (-vertical + horizontal));
*/
        }
    }
}
