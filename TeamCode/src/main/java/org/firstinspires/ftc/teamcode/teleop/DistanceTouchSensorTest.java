package org.firstinspires.ftc.teamcode.teleop;

import android.text.method.Touch;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;

@TeleOp(name="Sensor Test", group = "Subsystem Tests")
public class DistanceTouchSensorTest extends LinearOpMode {

    Mecanum robot;

    DigitalChannel distanceSensor;

    RevTouchSensor touchSensor;

    @Override
    public void runOpMode() throws InterruptedException {
//        robot = new Mecanum(hardwareMap);

        distanceSensor = hardwareMap.get(DigitalChannel.class, "distance");
        distanceSensor.setMode(DigitalChannel.Mode.INPUT);
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");

        waitForStart();

        while(!isStopRequested()){

//            robot.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);



            telemetry.addData("Distance (Dist Sensor): ",distanceSensor.getState());
            telemetry.addData("Touched (Touch Sensor): ",touchSensor.getValue());
            telemetry.update();
        }
    }
}
