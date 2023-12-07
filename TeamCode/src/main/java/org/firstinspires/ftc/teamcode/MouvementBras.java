package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MouvementBras extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotorEx bras1;
    private DcMotorEx bras2;

    private Servo coude;

    private Servo mains;

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coude = hardwareMap.get(Servo.class, "coude");
        mains = hardwareMap.get(Servo.class, "mains");

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

        double coudeX = 0;
        int brasA = 0;
        int brasB = 0;
        double trigger = 0;
        double varRY = 0;

        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            varY = this.gamepad1.left_stick_y;
            varX = this.gamepad1.left_stick_x;

            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);
            speed = Math.sqrt(varXpos * varXpos + varYpos * varYpos);

            varRY = this.gamepad1.right_stick_y;
            trigger = this.gamepad1.right_trigger;

            brasA = bras1.getCurrentPosition();
            brasB = bras2.getCurrentPosition();


            if (varY > 0) //Forward
            {
                tgtPowerA = varYpos;
                tgtPowerB = varYpos;

                if (varX < 0) {
                    tgtPowerA = tgtPowerA - varXpos;
                } else if (varX > 0) {
                    tgtPowerB = tgtPowerB - varXpos;
                }

            } else if (varY < 0) //Backward
            {
                tgtPowerA = -varYpos;
                tgtPowerB = -varYpos;

                if (varX < 0) {
                    tgtPowerA = tgtPowerA + varXpos;
                } else if (varX > 0) {
                    tgtPowerB = tgtPowerB + varXpos;
                }

            }
            //joyValue Between joyValueMidLower - joyValueMidUpper.
            //Need some range here, because joystick sometime not in  exact center.
            else if (varY == 0) {
                tgtPowerA = 0;
                tgtPowerB = 0;
            }

            if (varX > 0 && varY == 0) {
                tgtPowerA = -varXpos;
                tgtPowerB = varXpos;
            }

            if (varX < 0 && varY == 0) {
                tgtPowerA = varXpos;
                tgtPowerB = -varXpos;
            }


            if (this.gamepad1.left_bumper) {
                motorA.setPower(tgtPowerA);
                motorB.setPower(-tgtPowerB);
            } else {
                motorA.setPower((tgtPowerA / 2));
                motorB.setPower(-(tgtPowerB / 2));
            }

            if (varRY < 0) {
                bras1.setPower(varRY/2);
                bras2.setPower(varRY/2);
            } else {
                bras1.setPower(varRY/4);
                bras2.setPower(varRY/4);
            }

            if (this.gamepad1.dpad_up) {
                coudeX += 0.002;
                if (coudeX>1) {
                    coudeX = 1;
                }
            }
            if (this.gamepad1.dpad_down) {

                coudeX -= 0.002;
                if (coudeX<0) {
                    coudeX = 0;
                }
            }
            coude.setPosition(coudeX);

            mains.setPosition(trigger);

            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Var Y", varRY);
            telemetry.addData("Coude", coudeX);
            telemetry.addData("Bras", brasA);
            telemetry.addData("Main", trigger);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
