package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TestOpMode extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    //private DcMotor loading;
    //private DcMotor shoot;



    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        BackRight = hardwareMap.get(DcMotor.class, "backRight");
        //loading = hardwareMap.get(DcMotor.class, "load");
        //shoot = hardwareMap.get(DcMotor.class, "shoot");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double throttle = 0;
        double TurnL = 0;
        double TurnR = 0;
        boolean leftStrafe;
        boolean rightStrafe;
        boolean loader;
        boolean shooter;

        BackRight.setPower(0);
        BackLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        //loading.setPower(0);
        sleep(1000);

        while (opModeIsActive()) {
            throttle = this.gamepad1.left_stick_x;
            TurnL = this.gamepad1.right_stick_y;
            TurnR = -this.gamepad1.right_stick_y;
            leftStrafe = this.gamepad1.left_bumper;
            rightStrafe = this.gamepad1.right_bumper;
            loader = this.gamepad1.a;
            shooter = this.gamepad1.b;

            /*if (shooter) {
                shoot.setPower(-1);
            }else{
                shoot.setPower(0);
            }*/

            /*if (loader) {
                loading.setPower(.75);
                sleep(1000);
                loading.setPower(1);
            }else {
                loading.setPower(0);
            }*/

            BackLeft.setPower(throttle);
            BackRight.setPower(.75*throttle);
            frontLeft.setPower(.75*throttle);
            frontRight.setPower(.75*throttle);

            frontLeft.setPower(TurnL);
            frontRight.setPower(TurnR);
            BackRight.setPower(-TurnR);
            BackLeft.setPower(-TurnL);

//            armdrive.setPower(arm/2);

            if (leftStrafe){
                frontLeft.setPower(.5);
                frontRight.setPower(.5);
                BackLeft.setPower(-1);
                BackRight.setPower(-1);
            }

            if (rightStrafe) {
                frontLeft.setPower(-.5);
                frontRight.setPower(-.5);
                BackLeft.setPower(1);
                BackRight.setPower(1);
            }

            telemetry.addData("Target Power", throttle);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
