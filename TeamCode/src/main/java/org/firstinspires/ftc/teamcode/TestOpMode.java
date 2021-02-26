package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TestOpMode extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor loading;
    private DcMotor shoot;
    private DcMotor ramp;
    private DcMotor belt;



    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        BackRight = hardwareMap.get(DcMotor.class, "backRight");
        loading = hardwareMap.get(DcMotor.class, "load");
        shoot = hardwareMap.get(DcMotor.class, "shoot");
        ramp = hardwareMap.get(DcMotor.class, "ramp");
        belt = hardwareMap.get(DcMotor.class, "belt");


        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double turn = 0;
        double throttleL = 0;
        double throttleR = 0;
        boolean leftStrafe;
        boolean rightStrafe;
        boolean loader;
        boolean shooter;
        boolean AngleOfAttackUp;
        boolean AngleOfAttackDown;

        BackRight.setPower(0);
        BackLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        //loading.setPower(0);
        sleep(1000);

        while (opModeIsActive()) {
            turn = this.gamepad1.left_stick_x;
            throttleL = this.gamepad1.right_stick_y;
            throttleR = -this.gamepad1.right_stick_y;
            leftStrafe = this.gamepad1.left_bumper;
            rightStrafe = this.gamepad1.right_bumper;
            loader = this.gamepad1.a;
            shooter = this.gamepad1.b;
            AngleOfAttackUp = this.gamepad1.dpad_up;
            AngleOfAttackDown = this.gamepad1.dpad_down;

            if (shooter) {
                shoot.setPower(1);
            }else{
                shoot.setPower(0);
            }

            if (AngleOfAttackUp){
                ramp.setPower(.5);
            }else if (AngleOfAttackDown){
                ramp.setPower(-.25);
            }else{
                ramp.setPower(0);
            }

            if (loader) {
                loading.setPower(.75);
                sleep(1000);
                loading.setPower(1);
            }else {
                loading.setPower(0);
            }

            frontLeft.setPower(throttleL);
            frontRight.setPower(throttleR);
            BackRight.setPower(throttleR);
            BackLeft.setPower(throttleL);


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

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
