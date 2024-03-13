package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class CalibrageDirection extends LinearOpMode {

    private DcMotorEx motorA;
    private DcMotorEx motorB;

    private BNO055IMU gyro;

    private AnalogInput dist;

    private DcMotorEx bras1;
    private DcMotorEx bras2;

    private double zeroDuBras;
    private double zeroDuHaut;

    public void driveForwardPID(double distance, double power) {
        //normalSpeed();
        double ROTATIONS = distance / 0.2827;
        double COUNTS = ROTATIONS * 515.46;
        int leftTarget = (int) COUNTS - motorA.getCurrentPosition();
        int rightTarget = (int) COUNTS + motorB.getCurrentPosition();
        motorA.setTargetPosition(-leftTarget);
        motorB.setTargetPosition(rightTarget);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorA.setPower(0);
        motorB.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //resetMotors();
    }

    void goLeft(double power) {

        double ROTATIONS = 0.29 / 0.2827;
        double COUNTS = ROTATIONS * 515.46;
        int leftTarget = (int)  COUNTS + motorA.getCurrentPosition();
        int rightTarget = (int) COUNTS + motorB.getCurrentPosition();
        motorA.setTargetPosition(leftTarget);
        motorB.setTargetPosition(rightTarget);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorA.setPower(0);
        motorB.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void goRight(double power) {

        double ROTATIONS = 0.285 / 0.2827;
        double COUNTS = ROTATIONS * 515.46;
        int leftTarget = (int)  COUNTS - motorA.getCurrentPosition();
        int rightTarget = (int) -COUNTS + motorB.getCurrentPosition();
        motorA.setTargetPosition(-leftTarget);
        motorB.setTargetPosition(rightTarget);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorA.setPower(0);
        motorB.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void BrasGoTo(double pos) {
        if (bras2.getCurrentPosition() < zeroDuHaut+pos) {
            while (bras2.getCurrentPosition() < zeroDuHaut+pos) {
                bras1.setPower(0.3);
                bras2.setPower(0.3);
            }
            bras1.setPower(0);
            bras2.setPower(0);
        }
        else {
            while (bras2.getCurrentPosition() > zeroDuHaut+pos) {
                bras1.setPower(-0.3);
                bras2.setPower(-0.3);
            }
            bras1.setPower(0);
            bras2.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        //gyro = hardwareMap.get(BNO055IMU.class, "imu");
        //gyro.initialize(parameters);

        motorA = hardwareMap.get(DcMotorEx.class, "moteur1");
        motorB = hardwareMap.get(DcMotorEx.class, "moteur2");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        dist = hardwareMap.get(AnalogInput.class, "distanceS");

        double varY1;
        double tgtPowerA = 0;
        double tgtPowerB = 0;
        double att = 0;
        double quo = 1;
        zeroDuBras = bras2.getCurrentPosition();
        zeroDuHaut = zeroDuBras - 462;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            varY1 = this.gamepad1.left_stick_y;


            if (motorA.getVelocity() != 0) {
                quo = motorA.getVelocity() / motorB.getVelocity() * -1;
            }
            if (quo<0.4) {
                quo=0.4;
            }

            if (varY1 != 0) {

                motorA.setVelocity(varY1*2600);
                motorB.setVelocity(-motorA.getVelocity()*quo);

            } else {
                motorA.setVelocity(0);
                motorB.setVelocity(0);
            }

            */

            if (gamepad1.a) {
                //turnToRight();
                goLeft(1);
            }
            if (gamepad1.b) {
                //turnToRight();
                goRight(1);
            }




            telemetry.addData("Moteur 1 Puissance : ", motorA.getVelocity());
            //telemetry.addData("Moteur 1 difference : ", Math.abs(varY1)-Math.abs(tgtPowerA));
            telemetry.addData("Moteur 2 Puissance : ", motorB.getVelocity());

            telemetry.addData("Dist?", dist.getVoltage());
            telemetry.addData("Dist", dist.getConnectionInfo());
            telemetry.addData("PIDA", motorA.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("PIDB", motorB.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}