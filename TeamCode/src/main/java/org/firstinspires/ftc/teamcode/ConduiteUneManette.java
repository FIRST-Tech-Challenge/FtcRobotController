package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

        double coudeZero = 0.75;
        double coudeX = coudeZero;
        int brasA = 0;
        double triggergauche = 0;
        double triggerdroit = 0;
        double varRY = 0;
        double maPosCoude = 0;
        int posbraszero = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        posbraszero = bras1.getCurrentPosition();
        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            varY = this.gamepad1.left_stick_y;
            varX = this.gamepad1.left_stick_x;

            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);

            varRY = this.gamepad1.right_stick_y;

            brasA = bras1.getCurrentPosition();
            brasA = bras2.getCurrentPosition();

            triggerdroit = this.gamepad1.right_trigger;
            triggergauche = this.gamepad1.left_trigger;

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


            if (this.gamepad1.left_bumper) {
                motorA.setPower(tgtPowerA);
                motorB.setPower(-tgtPowerB);
            } else {
                motorA.setPower((tgtPowerA / 2));
                motorB.setPower(-(tgtPowerB / 2));
            }

            /// Bras + Coude + Main

            if (varRY < 0) {
                tgtBras = varRY/2;

            } else {
                tgtBras = varRY/4;
            }

            if (triggergauche > 0) {
                coudeX += 0.002;
                if (coudeX > 0.75) {
                    coudeX = 0.75;
                }
            } else if (triggerdroit > 0) {

                coudeX -= 0.002;
                if (coudeX<0) {
                    coudeX = 0;
                }
            }
            coude.setPosition(coudeX);


            if (this.gamepad1.x) {
                if (brasA > maPosBras) {
                    if (Math.abs(brasA-maPosBras)<30)
                    {
                        tgtBras = -0.2;
                    } else {
                        tgtBras = -0.5;
                    }

                } else if (brasA < maPosBras) {
                    if (Math.abs(brasA-maPosBras)>30)
                    {
                        tgtBras = 0.2;
                    } else {
                        tgtBras = 0.5;
                    }
                } else {
                    tgtBras = 0;
                }
                coudeX = maPosCoude;
            }

            if (this.gamepad1.y) {
                if (posbraszero < 0) {
                    posbraszero -= 2;
                } else {
                    posbraszero += 2;
                }
                bras1.setTargetPosition(posbraszero);
                bras2.setTargetPosition(posbraszero);
                coudeX = 0.384;
            }

            bras1.setPower(tgtBras);
            bras2.setPower(tgtBras);
            coude.setPosition(coudeX);

            if (mainG.getPosition() > 0.50) {
                while (this.gamepad1.a) {
                    mainG.setPosition(0);
                    mainD.setPosition(0.65);
                }}
            if (mainG.getPosition() < 0.20){
                while (this.gamepad1.a) {
                    mainG.setPosition(0.65);
                    mainD.setPosition(0);
                }
            }

            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Var Y", varRY);
            telemetry.addData("Coude", coudeX);
            telemetry.addData("Bras", brasA);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}
