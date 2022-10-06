package org.firstinspires.ftc.teamcode.OpModes;
//imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.new_robot;

@TeleOp(name = "22-23 TeleOp", group = "LinearOpMode")

public class new_teleop extends LinearOpMode {

    //tells you how long the robot has run for
    private ElapsedTime runtime = new ElapsedTime();


    new_robot robot = new new_robot();


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
            if(gamepad1.x) {
                robot.claw.setPosition(90);
            }
            if(gamepad2.y) {
                robot.claw.setPosition(0);
            }

            double verticalSliderPower =  gamepad1. left_stick_y;
            double horizontalSliderPower = gamepad1.left_stick_x;

            robot.verticalSlider.setPower(verticalSliderPower);
            robot.horizontalSlider.setPower(horizontalSliderPower);

        }

    }
}
