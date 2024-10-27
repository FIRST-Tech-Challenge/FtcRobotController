package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.ServoTester;

import java.util.ArrayList;

@TeleOp(name = "cool robot")
public class TestTeleOp extends LinearOpMode {
    DcMotorEx frontLeft,frontRight,backLeft,backRight;
    GamepadEvents controller1;

    double maxSpeed = 0.3;

     @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class,"FLM");
        backLeft = hardwareMap.get(DcMotorEx.class,"BLM");
        frontRight = hardwareMap.get(DcMotorEx.class,"FRM");
        backRight = hardwareMap.get(DcMotorEx.class,"BRM");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection((DcMotorSimple.Direction.REVERSE));
        controller1 = new GamepadEvents(gamepad1);

        while (!isStopRequested()) {

            //Input checks
            double forward = controller1.left_stick_y;
            double strafe = -controller1.left_stick_x;
            double rotate = -controller1.right_stick_x;
            //Activate Motors;
            frontLeft.setPower((forward + strafe + rotate) * maxSpeed);
            backLeft.setPower((forward - strafe + rotate) * maxSpeed);
            frontRight.setPower((forward - strafe - rotate) * maxSpeed);
            backRight.setPower((forward + strafe - rotate) * maxSpeed);

            //Display Telemetry information


            //Update controller information
            //Failing to add this into the event loop will mean
            //that all user inputs will not be read by the program
            controller1.update();
        }

    }
}
