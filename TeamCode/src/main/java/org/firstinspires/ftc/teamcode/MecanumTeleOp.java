package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MecanumTeleOp", group="test")
public class MecanumTeleOp extends LinearOpMode {

    FrenzyHardwareMap robot = new FrenzyHardwareMap();

    @Override
    public void runOpMode() throws InterruptedException {

        //import the hardware map

        robot.init(hardwareMap, telemetry);

        // init motor and add intake for arm
        robot.motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int armCurrentPos = 0;
        int armSetPos = 0;
        int armMaxPos = 500;

        //init loop
         while (! isStarted()) {

             telemetry.addData("Arm Encoder", robot.motorArm.getCurrentPosition());
             telemetry.addData("Say", "Hello Driver");
             telemetry.update();
         }


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            // Track Arm Current Pos
            armCurrentPos = robot.motorArm.getCurrentPosition();


            // Arm Up and Arm Down in small increments
            if (gamepad2.right_stick_y != 0.0 && ( armSetPos >= 0 && armSetPos <= armMaxPos) ) {
                if (gamepad2.right_stick_y == -1 ){
                    armSetPos = Math.min(++armSetPos, armMaxPos);
                } else if (gamepad2.right_stick_y == 1 ) {
                    armSetPos = Math.max(--armSetPos, 0);
                }
            }

            // Dpad initiates Arm Up and Resting positions
            if (gamepad2.dpad_up) {

                robot.motorArm.setTargetPosition(armSetPos);
                robot.motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.motorArm.setPower(0.3);

            } else if (gamepad2.dpad_down) {

                robot.motorArm.setTargetPosition(0);
                robot.motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.motorArm.setPower(0.3);

            }

            //intake code start

            //intake in
            if (gamepad2.left_trigger > 0){

                robot.motorIntake.setPower(0.75);
            }
             else if (gamepad2.left_trigger == 0){

                 robot.motorIntake.setPower(0);
            }

            //intake out
            if (gamepad2.right_trigger > 0){

                robot.motorIntake.setPower(-0.3);
            }
            else if (gamepad2.right_trigger == 0){

                robot.motorIntake.setPower(0);
            }



            //drivetrain mecanum
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.motorFrontLeft.setPower(frontLeftPower);
            robot.motorBackLeft.setPower(backLeftPower);
            robot.motorFrontRight.setPower(frontRightPower);
            robot.motorBackRight.setPower(backRightPower);


            telemetry.addData("Left Stick X", gamepad1.left_stick_y );
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y );
            telemetry.addData("Arm pos", armCurrentPos);
            telemetry.addData("Arm Set pos", armSetPos);
            telemetry.addData("Right Y Stick",gamepad1.right_stick_y);
            telemetry.addData("Left Y Stick",gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    //Method for arm level set position
    //change arm positions to fit max position set at top of code

    //why is areSetPos being passed in?
    public int armGoToPos (int armLevel, int armSetPos){
        /*if (armLevel == 0){
            armSetPos = 0;
        } else if (armLevel == 1) {
            armSetPos = 200;
        } else if (armLevel == 2) {
            armSetPos = 400;
        } else if (armLevel == 3) {
            armSetPos = 600;
        } else if (armLevel == 4) {
            armSetPos = 800;
        }*/
        return armSetPos;

    }
}