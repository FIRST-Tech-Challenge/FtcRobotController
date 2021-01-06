package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="testEncoder")
public class TestEncoder extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft" );
        frontRight = hardwareMap.get(DcMotor.class, "frontRight" );
        backLeft = hardwareMap.get(DcMotor.class, "backLeft" );
        backRight = hardwareMap.get(DcMotor.class, "backRight" );

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        frontLeft.setPower(.2);
        while (runtime.seconds()<2.0){
            updateTelemetry();
        }
        frontLeft.setPower(0);
        runtime.reset();
        frontRight.setPower(.2);
        while (runtime.seconds()<2.0){
            updateTelemetry();
        }
        frontRight.setPower(0);

        runtime.reset();
        backLeft.setPower(.2);
        while (runtime.seconds()<2.0){
            updateTelemetry();
        }
        backLeft.setPower(0);
        runtime.reset();
        backRight.setPower(0.2);
        while (runtime.seconds()<2.0){
            updateTelemetry();
        }
        backRight.setPower(0);

        wait(30_000);
    }

    protected void updateTelemetry(){
        telemetry.addData("Target Front Left Motor Position", frontLeft.getCurrentPosition());
        telemetry.addData("Target Front Right Motor Position", frontRight.getCurrentPosition());
        telemetry.addData("Target Back Left Motor Position", backLeft.getCurrentPosition());
        telemetry.addData("Target Back Right Motor Position", backRight.getCurrentPosition());
        telemetry.update();
    }
}
