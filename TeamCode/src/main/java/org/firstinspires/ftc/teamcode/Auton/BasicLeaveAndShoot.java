package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class BasicLeaveAndShoot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rightBack = hardwareMap.get(DcMotor.class ,"rb");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        DcMotor outtake = hardwareMap.get(DcMotor.class, "outtake");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");

        double power = 0.75;
        long strafeTime = 4000;
        long shootTime = 10000;
        waitForStart();
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(power);
            telemetry.addLine("strafing");
            telemetry.addData("power", power);
            telemetry.update();
            sleep(strafeTime);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            telemetry.addLine("finished strafing");
            telemetry.update();
            sleep(1000);
            outtake.setPower(0.85);
            intake.setPower(1);
            transfer.setPower(1);
            sleep(shootTime);
            outtake.setPower(0);
            intake.setPower(0);
            telemetry.addLine("pollen has been shot!");
            telemetry.update();
            //done
        }


    }
}
