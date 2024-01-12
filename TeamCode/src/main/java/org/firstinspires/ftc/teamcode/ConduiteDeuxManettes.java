package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Dictionary;
import java.util.Hashtable;

@TeleOp
public class ConduiteDeuxManettes extends LinearOpMode {
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

        double coudeZero = 0.5;
        double coudeX = coudeZero;
        double brasA = 0;
        double triggergauche = 0;
        double triggerdroit = 0;
        double varRY = 0;
        double maPosCoude = 0;
        int posbraszero = 0;
        int goTo = 0;
        Dictionary<String, Double> mesPosBras = new Hashtable();
        Dictionary<String, Double> mesPosCoude = new Hashtable();

        Gamepad manette1 = this.gamepad1;
        Gamepad manette2 = this.gamepad2;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        posbraszero = bras1.getCurrentPosition();
        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mesPosBras.put("x",0.0);
        mesPosBras.put("y",0.0);
        mesPosBras.put("a",0.0);
        mesPosBras.put("b",0.0);
        mesPosCoude.put("x",0.0);
        mesPosCoude.put("y",0.0);
        mesPosCoude.put("a",0.0);
        mesPosCoude.put("b",0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;

            varYpos = Math.abs(varY);
            varXpos = Math.abs(varX);

            varRY = manette2.right_stick_y;

            brasA = bras1.getCurrentPosition();
            brasA = bras2.getCurrentPosition();

            triggerdroit = manette2.right_trigger;
            triggergauche = manette2.left_trigger;

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
                manette1.rumble(100);
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

            coudeX += manette2.left_stick_y*0.02;
            coude.setPosition(coudeX);
            if (coudeX < 0) { coudeX = 0;}
            if (coudeX > 1) { coudeX = 1;}


            if (manette2.dpad_left) {
                mesPosBras.put("x",brasA);
                mesPosCoude.put("x",coude.getPosition());
            }
            if (manette2.dpad_right) {
                mesPosBras.put("b",brasA);
                mesPosCoude.put("b",coude.getPosition());
            }
            if (manette2.dpad_up) {
                mesPosBras.put("y",brasA);
                mesPosCoude.put("y",coude.getPosition());
            }
            if (manette2.dpad_down) {
                mesPosBras.put("a",brasA);
                mesPosCoude.put("a",coude.getPosition());
            }

            if (manette2.x) {
                maPosBras = mesPosBras.get("x");
                maPosCoude = mesPosCoude.get("x");
                goTo = 1;
            }
            else if (manette2.b) {
                maPosBras = mesPosBras.get("b");
                maPosCoude = mesPosCoude.get("b");
                goTo = 1;
            }
            else if (manette2.y) {
                maPosBras = mesPosBras.get("y");
                maPosCoude = mesPosCoude.get("y");
                goTo = 1;
            }
            else if (manette2.a) {
                maPosBras = mesPosBras.get("a");
                maPosCoude = mesPosCoude.get("a");
                goTo = 1;
            }
            else {goTo = 0;}

            if (goTo == 1) {
                if (brasA> maPosBras) {
                    tgtBras = -0.3;
                } else if (brasA < maPosBras) {
                        tgtBras = 0.3;
                } else {
                        tgtBras = 0;
                }
                coudeX = maPosCoude;
            }

            bras1.setPower(tgtBras);
            bras2.setPower(tgtBras);
            coude.setPosition(coudeX);

            if (mainG.getPosition() > 0.50) {
                while (manette2.left_bumper) {
                    mainG.setPosition(0);
                }
            }
            if (mainG.getPosition() < 0.20){
                while (manette2.left_bumper) {
                    mainG.setPosition(0.65);
                }
            }

            if (mainD.getPosition() > 0.50) {
                while (manette2.right_bumper) {
                    mainD.setPosition(0);
                }}
            if (mainD.getPosition() < 0.20){
                while (manette2.right_bumper) {
                    mainD.setPosition(0.65);
                }
            }

            telemetry.addData("Target Power A", tgtPowerA);
            telemetry.addData("Target Power B", tgtPowerB);
            telemetry.addData("Var Y", varRY);
            telemetry.addData("Coude", coudeX);
            telemetry.addData("Bras", brasA);
            telemetry.addData("Postion du bras eng", maPosBras);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
