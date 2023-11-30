package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MonPremierModeOp extends LinearOpMode {

    private DcMotor motorA;
    private DcMotor motorB;



    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPowerA = 0;
        double tgtPowerB = 0;

        double deadZone = 0.1;
        double varY = 0;
        double varX = 0;
        double varYpos = 0;
        double varXpos = 0;
        double speed = 0;

        while (opModeIsActive()) {
            varY = this.gamepad1.left_stick_y;
            varX = this.gamepad1.left_stick_x;

            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);
            speed = Math.sqrt(varXpos*varXpos+varYpos*varYpos);



            if (varY > 0) {
                if (varX > deadZone) {
                    tgtPowerA = speed;
                    tgtPowerB = -speed;
                } else if (varX < -deadZone) {
                    tgtPowerA = -speed;
                    tgtPowerB = speed;
                } else {
                    tgtPowerA = speed;
                    tgtPowerB = speed;
                }

            } else if (varY < 0) {
                if (varX > deadZone) {
                    tgtPowerA = -speed;
                    tgtPowerB = speed;
                } else if (varX < -deadZone) {
                    tgtPowerA = speed;
                    tgtPowerB = -speed;
                } else {
                    tgtPowerA = -speed;
                    tgtPowerB = -speed;
                }

            }

            if (varX==0 && varY==0) {
                tgtPowerA = 0;
                tgtPowerB = 0;
            }

            motorA.setPower(tgtPowerA);
            motorB.setPower(-tgtPowerB);

            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
