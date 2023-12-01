package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class CalibrageDirection extends LinearOpMode {

    private DcMotor motorA;
    private DcMotor motorB;



    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");
        double varY1;
        double tgtPowerA = 0;
        double tgtPowerB = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            varY1 = this.gamepad1.left_stick_y;


            motorA.setPower(varY1+tgtPowerA);
            motorB.setPower(-(varY1+tgtPowerB));

            if (this.gamepad1.a) {
                tgtPowerA -= 0.01;
            }
            if (this.gamepad1.b) {
                tgtPowerA += 0.01;
            }
            if (this.gamepad1.x) {
                tgtPowerB -= 0.01;
            }
            if (this.gamepad1.y) {
                tgtPowerB += 0.01;
            }


            telemetry.addData("Moteur 1 diff : ", tgtPowerA);
            telemetry.addData("Moteur 2 diff : ", tgtPowerB);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}