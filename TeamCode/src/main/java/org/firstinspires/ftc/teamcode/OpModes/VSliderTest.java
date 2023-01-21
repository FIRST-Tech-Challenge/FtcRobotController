package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
            if (gamepad2.a) {
                robot.vSlider.MoveSlider(1,1000,1000);
                sleep(1000);
                robot.vSlider.MoveSlider(1,-1000,1000);
            }

        }

    }

}
