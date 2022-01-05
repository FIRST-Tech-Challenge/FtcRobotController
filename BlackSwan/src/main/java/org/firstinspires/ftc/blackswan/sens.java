package org.firstinspires.ftc.blackswan;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp(name = "Test color sensors", group ="test")
public class sens extends LinearOpMode {

    Robot robot;

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Left Light Level: ", robot.colorSensor.alpha());
            telemetry.addData("Left Red", robot.colorSensor.red());
            telemetry.addData("Left Blue", robot.colorSensor.blue());
            telemetry.addData("Left Green", robot.colorSensor.green());

            telemetry.update();
        }
    }
}
