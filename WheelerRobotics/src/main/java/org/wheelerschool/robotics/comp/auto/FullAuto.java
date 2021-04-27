package org.wheelerschool.robotics.comp.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.comp.CompBot;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous
public class FullAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        CompBot bot = new CompBot(hardwareMap);
        BotNav nav = new BotNav(bot);

        nav.activate();

        waitForStart();

        // strafe sideways
        bot.setDriveEncTranslate(0.75f, 0, 500);

        while (!bot.atDriveTarget() && opModeIsActive()) {

        }

        bot.setDriveEncTranslate(0.75f, 1300, 0);

        while (!bot.atDriveTarget() && opModeIsActive()) {

        }

        bot.setDriveEncRotate(0.5f, -300);

        while (!bot.atDriveTarget() && opModeIsActive()) {

        }

        bot.launcher(true);

        bot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(
                !nav.moveTowardsTarget(new VectorF(-200, 1100, 0),
                new Orientation(EXTRINSIC, XYZ, RADIANS, 0, 0, (float) (Math.PI/2), 0))
                && opModeIsActive()
        ) {}

        bot.setDriveDirect(0,0,0,0);

        for (int i=0; i<4; i++) {
            bot.launchPush(true);
            runtime.reset();
            while (runtime.seconds() < 1) {}
            bot.launchPush(false);
            runtime.reset();
            while (runtime.seconds() < 1) {}
        }

        runtime.reset();
        while (runtime.seconds() < 1) {
            bot.jiggle();
        }
        bot.setDriveDirect(0,0,0,0);

        for (int i=0; i<4; i++) {
            bot.launchPush(true);
            runtime.reset();
            while (runtime.seconds() < 1) {}
            bot.launchPush(false);
            runtime.reset();
            while (runtime.seconds() < 1) {}
        }

        bot.setDriveEncTranslate(1.f, 400, 0);

        while (!bot.atDriveTarget()){}
    }
}
