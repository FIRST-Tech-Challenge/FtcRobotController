package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

@Autonomous(name = "auto", group = "autos")
public class blueAutoYellow extends LinearOpMode {

    public RRLocalizationRead localizationRead;
    public DcMotor frontL;
    public DcMotor backL;
    public DcMotor frontR;
    public DcMotor backR;



@Override
    public void runOpMode() {
        localizationRead = new RRLocalizationRead();
        localizationRead.initLocalization(hardwareMap);
        backL = hardwareMap.get(DcMotor.class, "backL");
        frontL = hardwareMap.get(DcMotor.class, "frontL");
        frontR = hardwareMap.get(DcMotor.class, "frontR");
        backR = hardwareMap.get(DcMotor.class, "backR");

        waitForStart();

        if (opModeIsActive()) {
            moveForwardByInches(10.0);  // Moves the robot forward by 10 inches
        }
    }

    public void moveForwardByInches(double inches) {
        // Record the initial position
        Vector2d initialPosition = localizationRead.returnPose().position;
        double targetY = initialPosition.y + inches;  // Assuming "forward" is along the y-axis

        // Move forward until we reach the target distance
        while (opModeIsActive() && (localizationRead.returnPose().position.y < targetY)) {
            backL.setPower(0);// Apply forward power
            backR.setPower(0);
            frontL.setPower(0);// Apply forward power
            frontR.setPower(0);
            localizationRead.returnPose();  // Update position each loop
        }

        // Stop the robot once the target position is reached
        backL.setPower(0);// Apply forward power
        backR.setPower(0);
        frontL.setPower(0);// Apply forward power
        frontR.setPower(0);
    }
}