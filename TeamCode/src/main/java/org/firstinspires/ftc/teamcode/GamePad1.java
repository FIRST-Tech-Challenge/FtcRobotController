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
    double pmodify = .35;

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

        if(gamepad1.right_trigger > .5) {
            pmodify = .25;
        }
        else if(gamepad1.right_bumper){
            pmodify = 1.5;
        }
        else{
            pmodify = 1;
        }


            robot.frontLeft.setPower(frontLeftPower * pmodify);
            robot.frontRight.setPower(frontRightPower * pmodify);
            robot.backLeft.setPower(backLeftPower * pmodify);
            robot.backRight.setPower(backRightPower * pmodify);




/////////////////////////////////////slide

        if(gamepad1.circle ){
            robot.claw.setPosition(.275);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setLevelUp(slideDown - 2100);
        }
        if (gamepad1.triangle ){
            robot.claw.setPosition(.05);
        }


        if (gamepad1.dpad_down && !robot.touchSensor.isPressed()) {//floor
            setLevelDown(slideDown + 23);

        }

        if (gamepad1.dpad_left) {//low
            setLevelDown(slideDown - 2100);
        }


        if (gamepad1.dpad_up) {/////////medium
            setLevelUp(slideDown - 3666);
        }

        if(gamepad1.dpad_right){
            setLevelDown(slideDown - 150);
        }


/////////////manual level set
        if(gamepad1.left_bumper) {
            robot.viperSlide.setPower(-.6);///up
        }
       else if(gamepad1.left_trigger >  .5 && !robot.touchSensor.isPressed()){///down
            robot.viperSlide.setPower(.6);
        }
        else{
            robot.viperSlide.setPower(0);
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
            robot.viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            robot.viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

}