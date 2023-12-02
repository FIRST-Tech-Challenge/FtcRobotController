package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

/**
 * Harry
 * test color sensor, must have dashboard tuneable ranges for each color, print out read values, which range it is in, include LED on the extended class test program
 */
@TeleOp(name = "RFColorSensorTest")
public class RFColorSensorTest extends LinearOpMode{
    RFColorSensor colorSensor;
    public void runOpMode() throws InterruptedException{
        BasicRobot robot = new BasicRobot(this, true);
        colorSensor = new RFColorSensor("colorSensor");
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive() && !isStopRequested()){
            packet.put("Color", colorSensor.getColor());
            packet.put("Dist(in.)", colorSensor.getDist());
            robot.update();
        }
    }
}