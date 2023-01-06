package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Gamepad0234", group = "Linear Opmode")
public class GamePad1 extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();
    ElapsedTime timer = new ElapsedTime();

    double robotAngle;
    double rightX;
    double h;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    int slideDown;

    @Override
    public void init() {
        robot.init(hardwareMap);
        slideDown = robot.viperSlide.getCurrentPosition();

    }

    @Override
    public void loop() {
//////////////////////////drive system
        telemetry.addData("SLIDE", "Current Position: %7d", robot.viperSlide.getCurrentPosition());
        telemetry.update();

        h = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;

        frontLeftPower = h * Math.sin(robotAngle) - rightX;
        frontRightPower = h * Math.cos(robotAngle) + rightX;
        backLeftPower = h * Math.cos(robotAngle) - rightX;
        backRightPower = h * Math.sin(robotAngle) + rightX;

        robot.frontLeft.setPower(frontLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backLeft.setPower(backLeftPower);
        robot.backRight.setPower(backRightPower);
/////////////////////////////////////slide
       // if (gamepad1.left_bumper) {//down halfway
         //   robot.viperSlide.setPower(.5);
        //}

        //if (gamepad1.left_trigger > .5) {//down all way
          //  robot.viperSlide.setPower(-.5);
        //}

        if(gamepad1.right_bumper){
        robot.claw1.setPosition(.3);
        }
        if(gamepad1.right_trigger >.5){
            robot.claw1.setPosition(1);
        }

        if (gamepad1.dpad_down){//floor
            setLevelDown(slideDown);
        }
        if (gamepad1.dpad_left) {//low
            setLevelDown(slideDown - 1100);
        }
        if (gamepad1.left_trigger > .5) {//down all way
            setLevelDown(slideDown + 23);

        }

        if (gamepad1.dpad_up) {/////////medium
            setLevelUp(slideDown - 1700);
        }

        if (gamepad1.right_trigger > .5) {//all the way up
            setLevelDown(slideDown - 3400);

        }

        if(gamepad1.dpad_right){
        robot.viperSlide.setPower(.5);
        }

        //////////////////////////////////////Levels

    }
        public void setLevelUp(int slideTarget){


            robot.viperSlide.setTargetPosition(slideTarget);
            //move the slide
            robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.viperSlide.setPower(.9);

            while (robot.viperSlide.isBusy()){
                telemetry.addData("SLIDE", "running to %7d : %7d",
                        slideTarget,
                        robot.viperSlide.getCurrentPosition());
                //telemetry.addData(slideDataSTR);
                telemetry.update();
            }
            robot.viperSlide.setPower(0);
        }
        public void setLevelDown(int slideTarget){

            robot.viperSlide.setTargetPosition(slideTarget);
            //move the slide


            robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.viperSlide.setPower(-.9);

            while (robot.viperSlide.isBusy()){
                telemetry.addData("SLIDE", "running to %7d : %7d",
                        slideTarget,
                        robot.viperSlide.getCurrentPosition());
                //telemetry.addData(slideDataSTR);
                telemetry.update();
            }
            robot.viperSlide.setPower(0);

        }

}