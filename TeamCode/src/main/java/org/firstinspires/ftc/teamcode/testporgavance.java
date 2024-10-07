package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testporgavance extends LinearOpMode {
    private DcMotor motorA;//le moteur A possede le scotch
    private DcMotor motorB;
    private DcMotorEx bras1;
    private DcMotorEx bras2;

    private Servo coudeG;

    private Servo coudeD;

    private Servo mainG;
    private Servo mainD;

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coudeG = hardwareMap.get(Servo.class, "coudeG");
        coudeD = hardwareMap.get(Servo.class, "coudeD");
        mainG = hardwareMap.get(Servo.class, "mainG");
        mainD = hardwareMap.get(Servo.class, "mainD");


        telemetry.addData("statut", "pret à lancé");
        telemetry.update();

        Gamepad manette1 = this.gamepad1;

        double varY = 0;
        double varX = 0;
        double varYpos = 0;
        double varXpos = 0;
        double tgtA;
        double tgtB;

        int brasA = 0;
        double triggergauche = 0;
        double triggerdroit = 0;
        double varRY = 0;
        double coudeZero = 0.91;
        double coudeX = coudeZero;
        double coudepas = 0.003;

        waitForStart();


        while (opModeIsActive()) {

            // Récupération valeur joystick gauche
            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;

            // Convertion pour Moteurs
            varYpos = abs(varY);
            varXpos = abs(varX);

            triggerdroit = this.gamepad1.right_trigger;
            triggergauche = this.gamepad1.left_trigger;
            varRY = this.gamepad1.right_stick_y;

            brasA = bras1.getCurrentPosition();

            tgtA = 0;
            tgtB = 0;

            if (varY > 0.05) {
                tgtA = varYpos;
                tgtB = varYpos;
            }
            if (varY < -0.05) {
                tgtA = -varYpos;
                tgtB = -varYpos;
            }
            if (varX > 0.05) {
                tgtA = -varXpos;
                tgtB = varXpos;
            }
            if (varX < -0.05) {
                tgtA = varXpos;
                tgtB = -varXpos;
            }
            if (varX > 0.05 && varY > 0.05) {
                tgtA = varXpos;
                tgtB = varXpos / 2;
            }
            if (varX < -0.05 && varY > 0.05) {
                tgtA = varXpos / 2;
                tgtB = varXpos;
            }
            if (varX > 0.05 && varY < -0.05) {
                tgtA = -varXpos;
                tgtB = -varXpos / 2;
            }
            if (varX < -0.05 && varY < -0.05) {
                tgtA = -varXpos / 2;
                tgtB = -varXpos;
            }

            motorA.setPower(tgtA);
            motorB.setPower(-tgtB);

            if (manette1.right_trigger > 0) {
                bras1.setPower(varRY / 2);
                bras2.setPower(varRY / 2);
            } else {
                bras1.setPower(varRY / 3);
                bras2.setPower(varRY / 3);
            }

            if (manette1.left_bumper) {
                coudeX += coudepas;
                if (coudeX > 0.83) {
                    coudeX = 0.83;
                }
                coudeG.setPosition(coudeX);

                coudeD.setPosition(1 - coudeX);
            }
            else if (manette1.right_bumper) {
                coudeX -= coudepas;
                if (coudeX<0.10) {
                    coudeX = 0.10;
                }
                coudeG.setPosition(coudeX);

                coudeD.setPosition(1 - coudeX);
            }

            if (mainG.getPosition() > 0.50) {
                while (manette1.a) {
                    mainG.setPosition(0);
                }
            }
            if (mainG.getPosition() < 0.20) {
                while (manette1.a) {
                    mainG.setPosition(1);
                }
            }

            if (mainD.getPosition() > 0.50) {
                while (manette1.b) {
                    mainD.setPosition(0);
                }
            }
            if (mainD.getPosition() < 0.20) {
                while (manette1.b) {
                    mainD.setPosition(1);
                }
            }

        }
    }
}
