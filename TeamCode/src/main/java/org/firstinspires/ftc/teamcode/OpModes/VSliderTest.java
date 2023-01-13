package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "VSlider Test", group = "LinearOpMode")

public class VSliderTest extends LinearOpMode {
    //tells you how long the robot has run for
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Instance of Robot class is initalized
         */
        robot.init(hardwareMap);

        /**
         * This code is run during the init phase, and when opMode is not active
         * i.e. When "INIT" Button is pressed on the Driver Station App
         */

        waitForStart();



        while (opModeIsActive()) {


            /**
             * Joystick controls for slider
             */

            // Testing slider, claw, and arm on gamepad2.
            // Sign of the power doesn't matter. Since the motor is reversed at initialization, the encoder values have to be negated.

            if(gamepad2.x) {
                robot.MoveSlider(0.5, 500,  1200);
                telemetry.addData("vSlider power 1 ", robot.vSlider.getPower());
                telemetry.addData("vSlider Encoder 1", robot.vSlider.getCurrentPosition());
                telemetry.addData("vSlider target 1", robot.vSlider.getTargetPosition());
                telemetry.update();




                robot.MoveSlider(0.5,  0, 1200);
                telemetry.addData("vSlider power 2", robot.vSlider.getPower());
                telemetry.addData("vSlider Encoder 2", robot.vSlider.getCurrentPosition());
                telemetry.addData("vSlider target 2", robot.vSlider.getTargetPosition());
                telemetry.update();


            }

            if(gamepad2.y) {

                timeout_ms = 600;
                runtime.reset();
                robot.vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.vSlider.setTargetPosition(500);
                robot.vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.vSlider.setPower(0.5);
                robot.vSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while ((runtime.milliseconds() < timeout_ms) && (robot.vSlider.isBusy())) {
                }
                robot.vSlider.setPower(0);

                telemetry.addData("vSlider power 1 ", robot.vSlider.getPower());
                telemetry.addData("vSlider Encoder 1", robot.vSlider.getCurrentPosition());
                telemetry.addData("vSlider target 1", robot.vSlider.getTargetPosition());
                telemetry.update();

                sleep(5000);



                runtime.reset();
                robot.vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.vSlider.setTargetPosition(-500);
                robot.vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.vSlider.setPower(0.5);
                robot.vSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while ((runtime.milliseconds() < timeout_ms) && (robot.vSlider.isBusy())) {
                }
                robot.vSlider.setPower(0);


                telemetry.addData("vSlider power 2 ", robot.vSlider.getPower());
                telemetry.addData("vSlider Encoder 2", robot.vSlider.getCurrentPosition());
                telemetry.addData("vSlider target 2", robot.vSlider.getTargetPosition());
                telemetry.update();



            }

            if(gamepad2.a) {
                for(int i = 0; i <= 20; i++) {
                    robot.SwingArmToPosition(1,65);
                    robot.swingArm.setPower(robot.swingArmHoldingPower);

                    robot.claw.setPosition(0);
                    sleep(500);
                    robot.claw.setPosition(1);

                    robot.SwingArmToPosition(-1,20);
                    robot.swingArm.setPower(0);
                }
            }



        }

    }

}
