

package org.firstinspires.ftc.teamcode.mentor.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name = "Mentor Arm PID", group = "Learning")

public class MentorArmPIDF extends LinearOpMode {

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

        // Setup the gamepad 2 buttons - Rename the variables to indicate gamepad 2
        boolean aPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;
        boolean bPressed = false;

        // Define arm PID and Velocity
        double armMaxVelocity = 3340;
        double F = 8; // Adjust until no more oscillations
        double P = 0; // 0.1 * F
        double I = 0; // 0.1 * P
        double D = 0;
        double positionP = 5.0;


        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        motorArm.setVelocityPIDFCoefficients(P, I, D, F);
        motorArm.setPositionPIDFCoefficients(positionP);

        motorArm.setTargetPositionTolerance(10);

        // Init Loop
        while( !isStarted() ){
            telemetry.addData("Arm Test", "Init Loop" );
            telemetry.addData("Arm Test", motorArm.getCurrentPosition() );
            telemetry.update();
        }

        while(opModeIsActive()) {

            /* Arm levels by Button
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
            motorArm.setVelocity(armMaxVelocity / 4); // This sets max velocity to 25%


            /* Intake In & Out
             * Gamepad 2
             * Left Trigger pulls in quickly
             * Right Trigger pushes out slowly
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