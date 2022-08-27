package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name= "Autonomous")

public class autonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        //create hardwareMap for this file
        HardwareMap hardwaremap = this.hardwareMap;
        //construct hardware class to call from
        hardware robot = new hardware();
        //calling the init functions to init the motors and sensor(you can do this in the constructor above as well, but in the HardwarePushbot the init is written in the init function)
        robot.init(this);
        //wait for the the user to press start button after robot initializes
        waitForStart();
        robot.moveForward();
        robot.isBlue();
        robot.moveBackward(robot.getRobotDistance()-robot.getSensorDistance());
        while(opModeIsActive()) {
            telemetry.addData("Sensor Distance", robot.getSensorDistance());
            telemetry.addData("Is Blue", robot.isBlue());
            telemetry.addData("robot distance", robot.getRobotDistance());
            telemetry.update();
        }
        //call functions, remember that color sensors are inaccurate from too far away or too close to the surface so make sure the robot is properly positioned before checking the color of the surface
        //Call functions from hardware class to do the task, you don’t have to create any functions other than specified for the task, but feel free to create others if you really need to


        stop();
    }
}
