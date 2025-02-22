package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled
public class RatchetServoTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IntakeClaw intakeClaw = IntakeClaw.getInstance(opModeUtilities);

        intakeClaw.getIntakeRatchetServo().setPosition(0.5);
        double ratchetServoPos = 0.5;

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                ratchetServoPos += 0.0001;
            } else if (gamepad1.y){
                ratchetServoPos -= 0.0001;
            }
            intakeClaw.getIntakeRatchetServo().setPosition(ratchetServoPos);

            telemetry.addData("ratchetServoPos", ratchetServoPos);
            telemetry.update();
        }

        //0.273 lock
        //0.135 push out
        //0.33 unlock


    }




}
