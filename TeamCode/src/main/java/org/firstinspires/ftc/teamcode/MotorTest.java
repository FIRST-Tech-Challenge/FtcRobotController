package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MotorTest")
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // declare motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); //motor direction
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Braking behavior
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //We don't want to use PID for the motors using the encoders

        waitForStart();


        if (isStopRequested()) return;


        while (opModeIsActive()) {

            motorFrontLeft.setPower(100);
        }
    }
}
