package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

import java.util.Random;

@TeleOp(name = "SlidesLimitSwitchTest")
public class SlidesLimitSwitchTest extends BaseTeleOp {
        Random rng = new Random();
        @Override
        public void runOpMode() {
            initialize();
            driveSlides(0);
            waitForStart();

            while (opModeIsActive()) {
                for(int i = 0; i <= 20; i++) {
                    for(int j = 0; j <= 5; j++) {
                        driveSlides(rng.nextInt(Constants.SLIDE_TOP-50)+50);
                    }
                    while(gamepad1.a) {

                    }
                    driveSlides(0);
                }

                for(Object o : telemetrySave) {
                    telemetry.addData("test", o);
                }
                telemetry.update();
                while(gamepad1.a && opModeIsActive()) {

                }
            }
        }
}
