package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "auto")
public class auto extends LinearOpMode {
    Drivetrain mecanum;
    DcMotorEx motorA, motorB;
    int currentPosition = 100;
    Servo claw1, claw2, wrist;
    final int tolerance = 10;
    @Override
    public void runOpMode() throws InterruptedException {

        mecanum = new Drivetrain(gamepad1, hardwareMap);
        motorA = hardwareMap.get(DcMotorEx.class, "A1");
        motorB = hardwareMap.get(DcMotorEx.class, "A2");

        claw1 = hardwareMap.get(Servo.class, "C1");
        claw2 = hardwareMap.get(Servo.class, "C2");
        wrist = hardwareMap.get(Servo.class, "W1");

        motorB.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime et = new ElapsedTime();




//        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        myArm = new Arm(hardwareMap, telemetry);
        telemetry.addData("Encoder PositionA", motorA.getCurrentPosition());

        waitForStart();

        motorA.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setTargetPositionTolerance(tolerance);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setTargetPosition(100);
        motorA.setPower(.5);

        motorB.setPower(0);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setTargetPositionTolerance(tolerance);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setTargetPosition(100);
        motorB.setPower(.5);
        et.reset();
        while (opModeIsActive()) {
            mecanum.drive(.2, .2, 0);
            if (et.seconds() > 3) {
                break;
            }
        }
    }
}
