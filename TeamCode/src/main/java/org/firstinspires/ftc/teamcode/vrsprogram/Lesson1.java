package org.firstinspires.ftc.teamcode.vrsprogram;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lesson1 (Blocks to Java)", group = "")
public class Lesson1 extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor ringShooter;
    private DcMotor ringFlywheel;

    /**
     * This function is executed when this Op Mode is selected.
     */
    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        ringShooter = hardwareMap.get(DcMotor.class, "ringShooter");
        ringFlywheel = hardwareMap.get(DcMotor.class, "ringFlywheel");

        // Put initialization blocks here.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.


            //Go forward
            frontLeft.setPower(1);
            frontRight.setPower(1);
            sleep(1850);

            //Turn right
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            sleep(200);

            //Stop moving
            frontLeft.setPower(0);
            frontRight.setPower(0);

            //Shoot
            ringFlywheel.setPower(1);
            sleep(200);
            ringShooter.setPower(1);
            sleep(2250);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            sleep(1000);
        }
    }
}


