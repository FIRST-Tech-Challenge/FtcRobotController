package org.wheelerschool.robotics.comp.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.comp.CompBot;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous
public class AutoSimple extends OpMode {
    CompBot bot;
    BotNav nav;

    @Override
    public void init() {
        bot = new CompBot(hardwareMap);
        nav = new BotNav(bot);
    }

    @Override
    public void start() {
        nav.activate();
    }

    @Override
    public void loop() {
        boolean onTarget = nav.moveTowardsTarget(new VectorF(0, 400, 0), new Orientation(EXTRINSIC, XYZ, RADIANS, 0, 0, (float) Math.PI, 0));
        telemetry.addData("Target visible", nav.botVis.targetVisible);
        telemetry.addData("On target", onTarget);

        if (nav.botVis.targetVisible) {
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    nav.botVis.getLastTranslation().get(0), nav.botVis.getLastTranslation().get(1), nav.botVis.getLastTranslation().get(2));
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", nav.botVis.getLastOrientation().firstAngle, nav.botVis.getLastOrientation().secondAngle, nav.botVis.getLastOrientation().thirdAngle);
        }
    }
}
