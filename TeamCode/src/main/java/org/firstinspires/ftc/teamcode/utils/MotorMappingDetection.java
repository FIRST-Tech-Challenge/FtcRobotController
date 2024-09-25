package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MacanumWheelsAuton;

@Autonomous(name="MotorMappingDetection", group="Furious Frog")
@Disabled
public class MotorMappingDetection extends LinearOpMode {
    /* Declare OpMode members. */
    DcMotor armMotor = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        MacanumWheelsAuton wheels = new MacanumWheelsAuton(hardwareMap, telemetry);
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        moveOneMotor(wheels, wheels.frontLeftMotor, "front left motor");//3
        moveOneMotor(wheels, wheels.backLeftMotor, "back left motor");//2
        moveOneMotor(wheels, wheels.backRightMotor, "back right motor");//0
        moveOneMotor(wheels, wheels.frontRightMotor, "front right motor");//1

//        moveOneMotor(wheels, armMotor, "arm motor");

        wheels.stop();
    }

    private void moveOneMotor(MacanumWheelsAuton wheels, DcMotor motor, String motorName) {
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Moving", motorName);
            motor.setPower(1);
            telemetry.update();
        }
        wheels.stop();
        runtime.reset();

        pause();
    }

    private void pause() {
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Status", "Pause");
            telemetry.update();
        }
        runtime.reset();
    }


}
