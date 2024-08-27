package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class SimpleDrive extends LinearOpMode {

    DcMotorEx frontRight, backRight, frontLeft, backLeft;

    public void runOpMode() {


        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        int driveSpeedIndex = 2; // change this to the index of the speed you want to start at
        double[] driveSpeedRange = {0.3, 0.6, 1};
        double driveSpeedFactor = 1;

        double rightPower = 0;
        double leftPower = 0;
        double slideToRight = 0;
        double slideToLeft = 0;



        boolean checkGOneRB = true;
        boolean checkGOneLB = true;




        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();


        while(opModeIsActive()) {

            // button checks

            if (!gamepad1.right_bumper) checkGOneRB = false;
            if (!gamepad1.left_bumper) checkGOneLB = false;



            // set drive speed
            if (gamepad1.right_bumper && !checkGOneRB && driveSpeedIndex < driveSpeedRange.length-1) {
                driveSpeedIndex++;


                driveSpeedFactor = driveSpeedRange[driveSpeedIndex];
                checkGOneRB = true;


            } else if (gamepad1.left_bumper && !checkGOneLB && driveSpeedIndex > 0) {
                driveSpeedIndex--;


                driveSpeedFactor = driveSpeedRange[driveSpeedIndex];
                checkGOneLB = true;
            }



            // assign power and set power
            rightPower = -gamepad1.right_stick_y * driveSpeedFactor;
            leftPower = -gamepad1.left_stick_y * driveSpeedFactor;

            slideToRight = -gamepad1.right_trigger * driveSpeedFactor * 1.1;
            slideToLeft = -gamepad1.left_trigger * driveSpeedFactor * 1.1;

            double denominator = Math.max(Math.abs((rightPower+leftPower)/2) + Math.abs(slideToRight) + Math.abs(slideToLeft), 1);

            frontRight.setPower((rightPower + slideToRight - slideToLeft) / denominator);
            backRight.setPower((rightPower - slideToRight + slideToLeft) / denominator);
            frontLeft.setPower((leftPower - slideToRight + slideToLeft)/ denominator);
            backLeft.setPower((leftPower + slideToRight - slideToLeft)/ denominator);



            // telemetry

            telemetry.addData("FR POW", frontRight.getPower());
            telemetry.addData("BR POW", backRight.getPower());
            telemetry.addData("FL POW", frontLeft.getPower());
            telemetry.addData("BL POW", backLeft.getPower());

            telemetry.addLine("--------------------------------");

            telemetry.addData("SPEED FACTOR", driveSpeedFactor);

            telemetry.update();


        }

    }
}
