package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Systems.Autonomous.*;
import org.firstinspires.ftc.teamcode.Systems.*;

import java.util.HashMap;
import java.util.Map;

class Auton extends Thread {

    RobotHardware rh;
    StandardTrackingWheelLocalizer stwl;
    Tensorflow tf; 

    Auton_Position robot;
    Threaded_Motor motor1;
    Threaded_Servo servo1;

    Updating_Telemetry telem;

    public boolean isRunning = true;

    public void run() { //MAIN FUNCTION
        robot = new Auton_Position(rh, stwl, 80.0, 70.0, 270.0);
        telem = new Updating_Telemetry(rh, stwl, tf);

        motor1 = new Threaded_Motor(rh, "motor name");
        servo1 = new Threaded_Servo(rh, "servo name");

        motor1.start();
        servo1.start();
        telem.start();

        robot.turn(90.00); //Should turn 90 degrees right
        robot.moveForward(100.0); //Should move 100 degrees forward
        robot.strafe(-100.0); //Should move 100 degrees Left

        telem.add_element_to_telemetry("this", "is how you add data to telemetry");

        pause(3000); //suspend reading of code

        telem.remove("this"); //is how you remove data from the telemetry
        telem.show_tensor = false;
        telem.add_element_to_telemetry("no longer showing", "tensorflow data");

        motor1.set_position(300);
        servo1.set_position(0.5);
        rh.setPower("motor 2", 0.3);

        waitFor(motor1); //waits for motor1 to finish

        isRunning = false;
    }

    public void quit() {
        motor1.should_be_running = false;
        servo1.should_be_running = false;
        telem.should_be_running = false;
        rh.stop();
    }

    public Auton(RobotHardware r) {
        rh = r;
    }

    public Auton(RobotHardware r, StandardTrackingWheelLocalizer l) {
        rh = r;
        stwl = l;
    }

    public Auton(RobotHardware r, Tensorflow t) {
        rh = r;
        tf = t;
    }

    public Auton(RobotHardware r, StandardTrackingWheelLocalizer l, Tensorflow t) {
        rh = r;
        stwl = l;
        tf = t;
    }

    public void waitFor(Threaded_Motor motor) {
        while (motor.isBusy) {
        }
    }

    public void pause(double milliseconds) {
        long t = System.nanoTime();
        while (System.nanoTime() - t <= milliseconds * 1000000) {
        }
    }
}

@Autonomous(name = "Autonomous Example")
public class Autonomous_Example extends LinearOpMode {

    RobotHardware r = new RobotHardware();
    StandardTrackingWheelLocalizer l;
    Tensorflow t;
    Auton a;

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap, telemetry);
        l = new StandardTrackingWheelLocalizer(hardwareMap);
        t = new Tensorflow(r);

        a = new Auton(r, l, t);

        waitForStart();

        a.start();

        while ((opModeIsActive()) && (a.isRunning)) {
            idle();
        }

        throw new IllegalArgumentException("lol");
        //a.quit();
        //stop();
    }

}

class Updating_Telemetry extends Thread {
    RobotHardware robot;
    StandardTrackingWheelLocalizer localizer;

    Pose2d currentPose;
    Tensorflow tensorflow;

    HashMap<String, String> telemetry = new HashMap<String, String>();

    public boolean should_be_running = true;
    boolean show_position;
    boolean show_tensor;

    public void add_element_to_telemetry(String key, String value) {
        telemetry.put(key, value);
    }

    public void remove(String key) {
        telemetry.remove(key);
    }

    public void run() {
        while (should_be_running) {
            localizer.update();
            currentPose = localizer.getPoseEstimate();
            robot.telemetry.clear();
            if (show_position) {
                robot.telemetry.addData("x", currentPose.getX());
                robot.telemetry.addData("y", currentPose.getY());
                robot.telemetry.addData("angle", currentPose.getHeading());
            }
            if (show_tensor) {
                for (String data : tensorflow.getData()) {
                    robot.telemetry.addData("tensorflow detects", data);
                }
            }
            for (Map.Entry<String, String> telem_elem : telemetry.entrySet()) {
                robot.telemetry.addData(telem_elem.getKey(), telem_elem.getValue());
            }
            robot.telemetry.update();
            pause(10);
        }
    }

    public void pause(double milliseconds) {
        long t = System.nanoTime();
        while (System.nanoTime() - t <= milliseconds * 1000000) {
        }
    }

    public Updating_Telemetry(RobotHardware r) {
        robot = r;
        show_position = false;
        show_tensor = false;
    }

    public Updating_Telemetry(RobotHardware r, StandardTrackingWheelLocalizer l) {
        robot = r;
        localizer = l;
        localizer.setPoseEstimate(l.getPoseEstimate());
        show_position = true;
        show_tensor = false;
    }

    public Updating_Telemetry(RobotHardware r, Tensorflow t) {
        robot = r;
        tensorflow = t;
        show_position = false;
        show_tensor = true;
    }

    public Updating_Telemetry(RobotHardware r, StandardTrackingWheelLocalizer l, Tensorflow t) {
        robot = r;
        localizer = l;
        localizer.setPoseEstimate(l.getPoseEstimate());
        tensorflow = t;
        show_position = true;
        show_tensor = true;
    }
}