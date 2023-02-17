package org.firstinspires.ftc.teamcode.Old.Teleop;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Old.Robots.FWDRobot;

@TeleOp(name = "MazeBotTeleOp")

public class MazeBotTeleOp extends LinearOpMode {
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;

    public void runOpMode() {

        FWDRobot MLO = new FWDRobot(this, true);

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        leftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorRightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        resetRuntime();
        while (!isStopRequested()&&getRuntime()<90) {
            float leftStickx = op.gamepad1.left_stick_x;
            float leftSticky = op.gamepad1.left_stick_y;
            float rightStick = gamepad1.right_stick_x;
            double leftSticktheta = atan2(leftSticky, leftStickx);
            double leftStickr = sqrt(pow(leftSticky, 2) + pow(leftStickx, 2));
            double powera = 0;
            double powerb = 0;
            double angle = atan2(leftSticky,leftStickx);

            powera = -sin(angle + PI/4);
            powerb = -sin(angle - PI/4);

//            if ((leftStickx + leftSticky)/(leftSticky - leftStickx) >= 1) {
//                powera = 1;
//                powerb = (leftSticky - leftStickx)/(leftStickx + leftSticky);
//            }
//            else if ((leftStickx + leftSticky)/(leftSticky - leftStickx) <= -1) {
//                powera = -1;
//                powerb = -(leftSticky - leftStickx)/(leftStickx + leftSticky);
//            }
//            else if ((leftStickx + leftSticky)/(leftSticky - leftStickx) > -1 && (leftStickx + leftSticky)/(leftSticky - leftStickx) < 0 || leftSticky-leftStickx < 0 && (leftStickx + leftSticky) == 0) {
//                powerb = -1;
//                powera = -(leftStickx + leftSticky)/(leftSticky - leftStickx);
//            }
//            else if ((leftStickx + leftSticky)/(leftSticky - leftStickx) < 1 && (leftStickx + leftSticky)/(leftSticky - leftStickx) > 0 || leftSticky-leftStickx > 0 && (leftStickx + leftSticky) == 0) {
//                powerb = 1;
//                powera = (leftStickx + leftSticky)/(leftSticky - leftStickx);
//            }


//        if (isSlow) {
//            if(abs(rightStick)<0.999&abs(rightStick)>0.2){
//                rightStick=0.999f*abs(rightStick)/rightStick;
//            }
//            motorLeftBack.setPower(rightSticky * 1.25/Math.sqrt(2));
//            motorRightFront.setPower((power - rightStick) * 0.3);
//            motorRightBack.setPower((power - rightStick) * 0.3);
//            motorLeftFront.setPower((power + rightStick) * 0.3);
//        }
//        else {
            if(abs(powera)>abs(powerb)
            ){
                powerb*=1/abs(powera);
                powera*=1/abs(powera);
            }
            else{
                powera*=1/abs(powerb);
                powerb*=1/abs(powerb);
            }
            leftFront.setPower((powerb * leftStickr));
            rightRear.setPower((powerb * leftStickr ));

            rightFront.setPower((powera * leftStickr));
            leftRear.setPower((powera * leftStickr ));

//        }
        }

        idle();
    }
}