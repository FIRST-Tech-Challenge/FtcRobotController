package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class testporg extends LinearOpMode {
    private DcMotor motorA;//le moteur A possede le scotch
    private DcMotor motorB;
 

    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");



        telemetry.addData("statut", "Attente du debut");
        telemetry.update();

        Gamepad manette1 = this.gamepad1;

        double varY = 0;
        double varX = 0;
        double varYpos = 0;
        double varXpos = 0;
        double tgtA;
        double tgtB;


        waitForStart();


        while (opModeIsActive()) {

            // Récupération valeur joystick gauche
            varY = manette1.left_stick_y;
            varX = manette1.left_stick_x;

            // Convertion pour Moteurs
            varYpos = abs(varY);
            varXpos = abs(varX);


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

            motorA.setPower(tgtA);
            motorB.setPower(-tgtB);
        }
    }
}