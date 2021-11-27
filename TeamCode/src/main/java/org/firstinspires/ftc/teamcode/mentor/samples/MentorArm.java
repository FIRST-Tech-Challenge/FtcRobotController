

package org.firstinspires.ftc.teamcode.mentor.samples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Mentor Arm Basic", group = "Learning")

public class MentorArm extends LinearOpMode {

    DcMotorEx motorArm;

    @Override
    public void runOpMode(){

        // init motors for Mentor Test - these are defined in the HardwareMap
        motorArm = hardwareMap.get(DcMotorEx.class, "arm");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        // Setup Arm Variables
        int armCurrentPos = 0;
        int armSetPos = 0;
        int armMaxPos = 700;
        int armLevel0 = 0;   // (a)
        int armLevel1 = 100; // (x)
        int armLevel2 = 400; // (y)
        int armLevel3 = 700; // (b)

        motorArm.setTargetPositionTolerance(10);
        motorArm.setTargetPosition(armSetPos);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setPower(0.3);

        // Init Loop
        while( !isStarted() ){
            telemetry.addData("Arm Test", "Init Loop" );
            telemetry.addData("Arm Test", motorArm.getCurrentPosition() );
            telemetry.update();
        }


        waitForStart();


        while(opModeIsActive()) {

            /* Arm levels by Button, Gamepad 2
             * a = 0    Resting position
             * x = 1    Level 1
             * y = 2    Level 2
             * b = 3    Level 3
             */
            if (gamepad2.a) {
                armSetPos = armLevel0;
            }else if (gamepad2.x) {
                armSetPos = armLevel1;
            }else if (gamepad2.y) {
                armSetPos = armLevel2;
            }else if (gamepad2.b) {
                armSetPos = armLevel3;
            }

            // Nudge arm Up and Arm Down in small increments
            if (gamepad2.right_stick_x != 0.0 && ( armSetPos >= 0 && armSetPos <= armMaxPos) ) {
                if (gamepad2.right_stick_x == -1 ){
                    armSetPos = Math.min(++armSetPos, armMaxPos);
                } else if (gamepad2.right_stick_x == 1 ) {
                    armSetPos = Math.max(--armSetPos, 0);
                }
            }

            motorArm.setTargetPosition(armSetPos);


            /* Intake In & Out by Triggers, Gamepad 2
             * Left Trigger pulls in quickly, 75% speed
             * Right Trigger pushes out slowly, 30% speed
             */
            // motorIntake.setPower(gamepad2.left_trigger * 0.75);
            // motorIntake.setPower(-gamepad2.right_trigger * 0.3);


            telemetry.addData("Arm Test", "Teleop Loop" );
            telemetry.addData("Arm Test", motorArm.getCurrentPosition() );
            telemetry.addData("Arm pos", armCurrentPos);
            telemetry.addData("Arm Set pos", armSetPos);
            telemetry.update();

        }

    }
}