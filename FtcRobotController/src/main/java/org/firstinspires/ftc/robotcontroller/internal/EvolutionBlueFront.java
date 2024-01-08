/* Copyright © 2023 North Paulding High School Robotics Team 16757 */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Evolution Blue Front", group="Evolution")
//@Disabled
public class EvolutionBlueFront extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDriveFront = null;
    private DcMotorEx rightDriveFront = null;
    private DcMotorEx leftDriveBack = null;
    private DcMotorEx rightDriveBack = null;

    private DcMotor planeLauncher = null;
    private Servo planeLoader = null;

    private DcMotor ziptie = null;
    private DcMotor lift = null;
    private Servo claw0 = null;
    private Servo claw1 = null;
    private double clawO = 1.0;

    private Servo pixel = null;

    private ColorSensor color = null;

    private double pos = -1;

    double[][] input;

    WolfNet network;

    @Override
    public void runOpMode() {
        leftDriveFront  = hardwareMap.get(DcMotorEx.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotorEx.class, "right_drive_front");
        leftDriveBack  = hardwareMap.get(DcMotorEx.class, "left_drive_back");
        rightDriveBack = hardwareMap.get(DcMotorEx.class, "right_drive_back");

        ziptie = hardwareMap.get(DcMotor.class, "ziptie");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw0 = hardwareMap.get(Servo.class, "claw0");
        claw1 = hardwareMap.get(Servo.class, "claw1");

        claw0 = hardwareMap.get(Servo.class, "claw0");
        claw1 = hardwareMap.get(Servo.class, "claw1");

        pixel = hardwareMap.get(Servo.class, "pixel");

        color = hardwareMap.get(ColorSensor.class, "color");

        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        claw0.setPosition(1.0f);
        claw1.setPosition(0.0f);

        network = new WolfNet(12, 8, 2, "Weights0", 0.5);
        network.LoadWeights();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            input = new double[][]{{-1.0, 1.0, runtime.time(), leftDriveFront.getVelocity(), rightDriveFront.getVelocity(), leftDriveBack.getVelocity(), rightDriveBack.getVelocity(), clawO, lift.getPower(), ziptie.getPower(), color.blue(), pos}};
            network.GetOutput(input);

            drive(network.output.layer[0][0], network.output.layer[0][1], network.output.layer[0][2]);

            if(network.output.layer[0][3] >= 0){
                claw0.setPosition(1.0f);
                claw1.setPosition(0.0f);
                clawO = 1;
            }else{
                claw0.setPosition(0.25f);
                claw1.setPosition(0.75f);
                clawO = -1;
            }

            lift.setPower(network.output.layer[0][4]);
            ziptie.setPower(network.output.layer[0][5]);

            pixel.setPosition(network.output.layer[0][6]);

            pos = network.output.layer[0][7];
        }
    }

    void drive(double x1, double x2, double y1){
        double fl = 0.0;
        double fr = 0.0;
        double bl = 0.0;
        double br = 0.0;

        fl += y1;
        fr += y1;
        bl += y1;
        br += y1;

        fl -= x1;
        fr += x1;
        bl += x1;
        br -= x1;

        fl -= x2;
        fr += x2;
        bl -= x2;
        br += x2;

        leftDriveFront.setVelocity(fl * 1000);
        rightDriveFront.setVelocity(fr * 1000);
        leftDriveBack.setVelocity(bl * 1000);
        rightDriveBack.setVelocity(br * 1000);
    }
}