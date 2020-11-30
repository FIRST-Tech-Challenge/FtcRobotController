package teleop;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import global.TerraBot;
@Disabled
@TeleOp(name = "TestOp V1")
public class TestOp extends OpMode {

    TerraBot bot = new TerraBot();




    @Override
    public void init() {

        telemetry.addData("Status: ","Not Ready");
        telemetry.update();

        bot.init(hardwareMap);

        telemetry.addData("Status: ","Ready");
        telemetry.update();







    }


    @Override
    public void start() {
        bot.startOdoThreadTele();
    }

    @Override
    public void loop() {

        if(gamepad1.y){
            bot.calibrateCol.start();
        }
//
//        if(gamepad1.y){
//            bot.calibrate.start();
//        }
//        if(gamepad1.x){
//            bot.heading = bot.localizer.getCalibratedTheta();
//            bot.lastAngle = bot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        }

//        telemetry.addData("l2", bot.localizer.l2);
//        telemetry.addData("angle", Math.toDegrees(bot.localizer.angle));
//        telemetry.addData("lastl2", bot.localizer.lastL2);
//        telemetry.addData("lastx", bot.localizer.lastX);
//        telemetry.addData("lasty", bot.localizer.lastY);
//        telemetry.addData("lastAngle", bot.localizer.lastAngle);
//        telemetry.addData("sub", bot.localizer.sub);
//        telemetry.addData("extFist", bot.localizer.extFirst);
//        telemetry.addData("extSecond", bot.localizer.extSecond);
//        telemetry.addData("cx", bot.localizer.cx);
//        telemetry.addData("cy", bot.localizer.cy);
//        telemetry.addData("c", bot.localizer.c);
//        telemetry.addData("the", Math.toDegrees(bot.localizer.the));
//        telemetry.addData("cal", bot.localizer.getCalibratedTheta());

//        telemetry.addData("heading", bot.getHeading());
////        telemetry.addData("l2", bot.getDisL2());
//
//
//
        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;
//
//        if(!bot.calibrate.executing) {
//            bot.moveTeleOp(forward, strafe, turn);
//        }
//
//        bot.update();

//        telemetry.addData("cor", bot.isOnWhiteR());
//        telemetry.addData("col", bot.isOnWhiteL());
//        telemetry.addData("corV", bot.getColorR()[2]);
//        telemetry.addData("colV", bot.getColorL()[2]);
//        telemetry.update();

        if(!bot.calibrateCol.executing) {
            bot.moveTeleOp(forward, strafe, turn);
        }

        telemetry.addData("heading", bot.getHeading());

        bot.update();



    }

    @Override
    public void stop() {
        bot.stopOdoThreadTele();
    }
}
