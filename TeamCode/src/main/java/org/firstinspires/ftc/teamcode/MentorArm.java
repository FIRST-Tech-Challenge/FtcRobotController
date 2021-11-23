package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Mentor Basic Arm", group = "Learning")

public class MentorArm extends LinearOpMode {

    DcMotorEx motorArm;

    @Override
    public void runOpMode(){

        // init motors
        motorArm = hardwareMap.get(DcMotorEx.class, "arm");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        int armCurrentPos = 0;
        int armSetPos = 0;
        int armMaxPos = 700;
        int armLevel0 = 0;   // (a)
        int armLevel1 = 100; // (x)
        int armLevel2 = 400; // (y)
        int armLevel3 = 700; // (b)
        boolean aPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;
        boolean bPressed = false;

        double armMaxVelocity = 3340.0;
        double F = 8; // Adjust until no more oscillations
        double P = 0; // 0.1 * F
        double I = 0; // 0.1 * P
        double D = 0;
        double positionP = 5.0;

        motorArm.setVelocityPIDFCoefficients(P, I, D, F);
        motorArm.setPositionPIDFCoefficients(positionP);

        motorArm.setTargetPositionTolerance(10);
        motorArm.setTargetPosition(armSetPos);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(3340 / 4);

        telemetry.addData("Arm Test", "Init" );
        telemetry.addData("Arm Test", motorArm.getCurrentPosition() );
        telemetry.update();

        // Init Loop
        while( !isStarted() ){
            telemetry.addData("Arm Test", "Init Loop" );
            telemetry.addData("Arm Test", motorArm.getCurrentPosition() );
            telemetry.update();
        }

        while(opModeIsActive()) {

            // Arm levels by Button : a = 0, x = 1, y = 2, b = 3
            if (gamepad2.a & !aPressed) {
                armSetPos = armLevel0;
            }
            aPressed = gamepad2.a;

            if (gamepad2.x & !xPressed) {
                armSetPos = armLevel1;
            }
            xPressed = gamepad2.x;

            if (gamepad2.y & !yPressed) {
                armSetPos = armLevel2;
            }
            yPressed = gamepad2.y;

            if (gamepad2.b & !bPressed) {
                armSetPos = armLevel3;
            }
            bPressed = gamepad2.b;

            // Nudge arm Up and Arm Down in small increments
            if (gamepad2.right_stick_x != 0.0 && ( armSetPos >= 0 && armSetPos <= armMaxPos) ) {
                if (gamepad2.right_stick_x == -1 ){
                    armSetPos = Math.min(++armSetPos, armMaxPos);
                } else if (gamepad2.right_stick_x == 1 ) {
                    armSetPos = Math.max(--armSetPos, 0);
                }
            }

            motorArm.setTargetPosition(armSetPos);


            //intake in
            // motorIntake.setPower(gamepad2.left_trigger * 0.75);

            //intake out
            // motorIntake.setPower(-gamepad2.right_trigger * 0.3);


            telemetry.addData("Arm Test", "Teleop Loop" );
            telemetry.addData("Arm Test", motorArm.getCurrentPosition() );
            telemetry.addData("Right Stick X", gamepad2.right_stick_x );
            telemetry.addData("Right Stick Y", gamepad2.right_stick_y );
            telemetry.addData("DPad Up", gamepad2.dpad_up );
            telemetry.addData("DPad Down", gamepad2.dpad_down );
            telemetry.addData("Arm pos", armCurrentPos);
            telemetry.addData("Arm Set pos", armSetPos);
            telemetry.update();

        }

    }
}