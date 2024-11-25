package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Testtest2")
public class Test extends LinearOpMode {
    // motor declaration, we use the
    // Ex version as it has velocity measurements
    DcMotorEx motor1, motor2;

    void extendArm(double input){
        motor2.setPower(input);
        motor1.setPower(input);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // the string is the hardware map name
        motor1 = hardwareMap.get(DcMotorEx.class, "m3");
        motor2 = hardwareMap.get(DcMotorEx.class,"m4");
        // use braking to slow the motor down faster
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        //  motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // loop that runs while the program should run.
        while (opModeIsActive()) {
            extendArm(-gamepad1.left_stick_y);
        }
    }
}