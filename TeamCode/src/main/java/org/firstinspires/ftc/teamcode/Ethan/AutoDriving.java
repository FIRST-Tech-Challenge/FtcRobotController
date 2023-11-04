package org.firstinspires.ftc.teamcode.Ethan;

import static com.qualcomm.robotcore.hardware.DcMotor.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp
public class AutoDriving extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lFront = hardwareMap.dcMotor.get("Left front");
        DcMotor rFront = hardwareMap.dcMotor.get("Right front");
        DcMotor lBack = hardwareMap.dcMotor.get("Left back");
        DcMotor rBack = hardwareMap.dcMotor.get("Right back");


        rBack.setDirection(Direction.FORWARD);
        lBack.setDirection(Direction.REVERSE);
        rFront.setDirection(Direction.FORWARD);
        lFront.setDirection(Direction.REVERSE);


        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFront.setMode(RunMode.STOP_AND_RESET_ENCODER);
        lFront.setMode(RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();

        int ticksTurned = lFront.getCurrentPosition();
        telemetry.addLine(String.valueOf(ticksTurned));
        telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));


        while(opModeIsActive()){

            while (ticksTurned < 383) {

                ticksTurned = lFront.getCurrentPosition();

                telemetry.addLine(String.valueOf(ticksTurned));
                telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));
                telemetry.update();

                lFront.setPower(1);
                lBack.setPower(1);
                rFront.setPower(1);
                rBack.setPower(1);
                if (ticksTurned >= 383) {
                    break;
                }
            }
            if (ticksTurned >= 383) {

                lFront.setPower(0);
                lBack.setPower(0);
                rFront.setPower(0);
                rBack.setPower(0);
            }
        }
    }
}