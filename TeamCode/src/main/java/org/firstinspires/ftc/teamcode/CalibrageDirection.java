package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class CalibrageDirection extends LinearOpMode {

    private DcMotorEx motorA;
    private DcMotorEx motorB;

    private BNO055IMU gyro;



    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        motorA = hardwareMap.get(DcMotorEx.class, "moteur1");
        motorB = hardwareMap.get(DcMotorEx.class, "moteur2");
        double varY1;
        double tgtPowerA = 0;
        double tgtPowerB = 0;
        double att = 0;
        double quo = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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

            telemetry.addData("Moteur 1 Puissance : ", motorA.getVelocity());
            //telemetry.addData("Moteur 1 difference : ", Math.abs(varY1)-Math.abs(tgtPowerA));
            telemetry.addData("Moteur 2 Puissance : ", motorB.getVelocity());
            telemetry.addData("Quo", quo);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}