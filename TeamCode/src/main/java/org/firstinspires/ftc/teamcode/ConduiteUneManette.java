package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ConduiteUneManette extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotorEx bras1;
    private DcMotorEx bras2;

    private Servo coude;

    private Servo mainG;
    private Servo mainD;

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coude = hardwareMap.get(Servo.class, "coude");
        mainG = hardwareMap.get(Servo.class, "mainG");
        mainD = hardwareMap.get(Servo.class, "mainD");

        double tgtPowerA = 0;
        double tgtPowerB = 0;

        double tgtBras = 0;
        double maPosBras = 0;

        double varY = 0;
        double varX = 0;
        double varYpos = 0;
        double varXpos = 0;

        double coudeZero = 0.91;
        double coudeX = coudeZero;
        int brasA = 0;
        double triggergauche = 0;
        double triggerdroit = 0;
        double varRY = 0;
        double maPosCoude = 0;
        int bras0 = 0;

        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;

            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);

            varRY = manette1.right_stick_y;

            brasA = bras1.getCurrentPosition();
            brasA = bras2.getCurrentPosition();

            triggerdroit = manette1.right_trigger;
            triggergauche = manette1.left_trigger;

            /// Mouvements
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


            if (manette1.left_bumper) {
                motorA.setPower(tgtPowerA);
                motorB.setPower(-tgtPowerB);
            } else {
                motorA.setPower((tgtPowerA / 2));
                motorB.setPower(-(tgtPowerB / 2));
            }

            /// Bras + Coude + Main

            if (varRY < 0) {
                tgtBras = varRY/3;
                bras0 = brasA;

            } else if (varRY > 0) {
                tgtBras = varRY/3;
                bras0 = brasA;
            } else {
                if (brasA > bras0) {
                    tgtBras = -0.1;
                } else if (brasA < bras0) {
                    tgtBras = 0.1;
                } else {
                    tgtBras = 0;
                }
            }

            if (triggergauche > 0) {
                coudeX += 0.0013;
                if (coudeX > 0.91) {
                    coudeX = 0.91;
                }
            } else if (triggerdroit > 0) {

                coudeX -= 0.0013;
                if (coudeX<0) {
                    coudeX = 0;
                }
            }
            coude.setPosition(coudeX);


            if (manette2.x) {
                if (brasA > maPosBras) {
                    tgtBras = -0.5;
                } else if (brasA < maPosBras) {
                    tgtBras = 0.5;
                } else {
                    tgtBras = 0;
                }
                coudeX = maPosCoude;
            }

            if (manette2.y) {
                maPosBras = brasA;
                maPosCoude = coude.getPosition();
            }

            bras1.setPower(tgtBras);
            bras2.setPower(tgtBras);
            coude.setPosition(coudeX);

            if (mainG.getPosition() > 0.10) {
                while (manette2.a) {
                    mainG.setPosition(0);
                }}
            if (mainG.getPosition() < 0.2){
                while (manette2.a) {
                    mainG.setPosition(0.4);
                }
            }

            if (mainD.getPosition() > 0.6) {
                while (manette2.b) {
                    mainD.setPosition(0.45);
                }}
            if (mainD.getPosition() < 0.5){
                while (manette2.b) {
                    mainD.setPosition(1);
                }
            }

            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Var Y", varRY);
            telemetry.addData("Coude", coudeX);
            telemetry.addData("Bras", brasA);
            telemetry.addData("Postion du bras eng", maPosBras);
            telemetry.addData("bras0", bras0);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
