package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.modules.RevDistance;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class RevDistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Rev2mDistanceSensor revDistance = hardwareMap.get(Rev2mDistanceSensor.class, "revDistance");
        Rev2mDistanceSensor revDistance2 = hardwareMap.get(Rev2mDistanceSensor.class, "revDistance2");
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(revDistance.getDistance(DistanceUnit.MM) + "");
            telemetry.addLine(revDistance2.getDistance(DistanceUnit.MM) + "");
            if ((revDistance.getDistance(DistanceUnit.MM) < 45)) {
                outtake.getOuttakeClaw().setPosition(Outtake.OUTTAKE_CLAW_CLOSE);
                telemetry.addLine("sample grabbed");
            }

            else if ((revDistance2.getDistance(DistanceUnit.MM) < 100)) {
                outtake.getOuttakeClaw().setPosition(Outtake.OUTTAKE_CLAW_CLOSE);
                telemetry.addLine("specimen ready");
            }

            telemetry.update();
        }
    }
}
