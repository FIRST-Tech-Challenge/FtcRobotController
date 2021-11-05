package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    FrenzyBot bot = new FrenzyBot();

    private static final String TAG = "MaxVelocityTest";

    @Override
    public void runOpMode() {
        try {

            bot.init(this, this.hardwareMap, telemetry);

            waitForStart();
            double maxLF = 0, maxLB = 0, maxRF = 0, maxRB = 0;

            bot.moveAtMaxSpeed();

            while (opModeIsActive()) {

                double LF = bot.getLeftVelocity();
                double RF = bot.getRightVelocity();
                double LB = bot.getLeftBackVelocity();
                double RB = bot.getRightBackVelocity();

                if ( LF > maxLF) {
                    maxLF = LF;
                }

                if ( RF > maxRF) {
                    maxRF = RF;
                }

                if ( LB > maxLB) {
                    maxLB = LB;
                }

                if ( RB > maxRB) {
                    maxRB = RB;
                }

                telemetry.addData("Max velocity LF", maxLF);
                telemetry.addData("Max velocity RF", maxRF);
                telemetry.addData("Max velocity LB", maxLB);
                telemetry.addData("Max velocity RB", maxRB);
                telemetry.update();
            }

            Log.d(TAG, String.format("maxLF: %.2f, maxRF: %.2f, maxLB: %.2f, maxRB: %.2f", maxLF, maxRF, maxLB, maxRB));

        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            bot.stop();
        }
    }
}

