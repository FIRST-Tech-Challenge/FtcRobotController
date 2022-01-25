package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestTwo extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");

        /** Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone). */
        DcMotor duckWheel = hardwareMap.get(DcMotor.class, "duckWheel");
        DcMotorEx frontL  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx frontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx backL  = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx backR = hardwareMap.get(DcMotorEx.class, "rightRear");
//        DcMotor intakeL = hardwareMap.get(CRServo.class, "intakeL");
//        DcMotor intakeR = hardwareMap.get(CRServo.class, "intakeR");
        DcMotorEx extender = hardwareMap.get(DcMotorEx.class, "extender");
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");

        // Reset Encoder
        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        /* Sets the motors to run using encoders. */




        /* Most robots need the motor on one side to be reversed to drive forward.
         * Reverse the motor that runs backwards when connected directly to the battery. */
        frontL.setDirection(DcMotorEx.Direction.FORWARD);
        backL.setDirection(DcMotorEx.Direction.FORWARD);
        frontR.setDirection(DcMotorEx.Direction.REVERSE);
        backR.setDirection(DcMotorEx.Direction.REVERSE);
//        intakeL.setDirection(CRServo.Direction.REVERSE);
//        CRServo intakeR.setDirection(CRServo.Direction.FORWARD);
//        DcMotor duckWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Full revolution 384.5(PPR)
        int power = 100;
        int targetPosition = 384;


        waitForStart();
        while(!gamepad1.left_bumper) {
            while (!gamepad1.right_bumper) {
                sleep(100);
                if (gamepad1.a) {
                    power += 10;

                }
                if (gamepad1.b) {
                    power -= 10;

                }
                if (gamepad1.x) {
                    targetPosition += 50;

                }
                if (gamepad1.y) {
                    targetPosition -= 50;

                }
                telemetry.addData("Target Position: ", targetPosition);
                telemetry.addData("Power: ", power);
                telemetry.update();
            }

            // Run While the Autonomous Mode is Active


            // Update telemetry status to show that it is running
            telemetry.addData("Status", "Running");
            telemetry.update();

            frontL.setTargetPosition(-targetPosition);
            frontR.setTargetPosition(targetPosition);
            backL.setTargetPosition(-targetPosition);
            backR.setTargetPosition(targetPosition);

            frontL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontL.setVelocity(power);
            frontR.setVelocity(power);
            backL.setVelocity(power);
            backR.setVelocity(power);



            while (frontL.isBusy() || frontR.isBusy() || backL.isBusy() || backR.isBusy()) {
                telemetry.addData("Status", "Waiting for Motors");
                telemetry.addData("Motors", "frontL Position: %d", frontL.getCurrentPosition());
                telemetry.addData("Motors", "frontR Position: %d", frontR.getCurrentPosition());
                telemetry.addData("Motors", "backL Position: %d", backL.getCurrentPosition());
                telemetry.addData("Motors", "backR Position: %d", backR.getCurrentPosition());
                telemetry.update();
            }
            // Stop the Autonomous Mode after we finish parking
        }
    }
}
