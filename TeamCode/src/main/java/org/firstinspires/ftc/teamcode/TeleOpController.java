package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Controller", group = "Linear OpMode")

public class TeleOpController extends LinearOpMode {

    // Declarare Motoare

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private ElapsedTime tick = new ElapsedTime();

    @Override
    public void runOpMode() {

        // START
        initMotoare();
        telemetry.addData("Starting at",  "FRONT: %7d :%7d | BACK: %7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition()
        );
        telemetry.update();
        waitForStart();


        // RUNNING

        while (opModeIsActive()){
            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + tick.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Coordinates ",  "FRONT: %7d :%7d | BACK: %7d :%7d",
                    leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition()
            );
            telemetry.update();
        }


    }

    // Asociaza componentele inainte de start
    public void initMotoare(){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
}
