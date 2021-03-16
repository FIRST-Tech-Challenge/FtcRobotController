package org.wheelerschool.robotics.comp.auto;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.comp.BotVision;
import org.wheelerschool.robotics.comp.CompBot;

public class BotNav {
    CompBot bot;
    public BotVision botVis;

    private float maxPower = 0.75f;
    private float targetDeadband = 50; // mm
    private float targetRamp = 500; // mm
    private float angleDeadband = (float) Math.PI / 12; // rad
    private float angleRamp = (float) Math.PI / 4; // rad

    public BotNav(CompBot bot) {
        this.bot = bot;
        botVis = new BotVision(bot);
    }

    public void activate() {
        botVis.activate();
    }

    // A correct modulo function:
    private static double positiveModulo(double numb, double modulo) {
        double n = numb % modulo;
        if (n < 0) {
            n += modulo;
        }
        return n;
    }

    // Calculate the difference between two angles (rad):
    public static double angleDifference(double angle1, double angle2) {
        return positiveModulo(((angle1 - angle2) + Math.PI), (2 * Math.PI)) - Math.PI;
    }

    private static VectorF getTargetDelta(VectorF current, VectorF target) {
        return target.subtracted(current);
    }

    private static VectorF rotateToFrame(VectorF delta, float angle) {
        MatrixF rotMatrix = new GeneralMatrixF(3, 3,
                new float[] {(float) Math.cos(angle), (float) -Math.sin(angle), 0,
                        (float) Math.sin(angle), (float) Math.cos(angle), 0, 0, 0, 0});

        return rotMatrix.multiplied(delta);
    }

    public boolean moveTowardsTarget(VectorF target, Orientation angle) {
        botVis.getLocation();

        boolean onTarget = false;


        if (botVis.targetVisible) {

            VectorF delta = getTargetDelta(botVis.getLastTranslation(), target);
            VectorF robotDelta = rotateToFrame(delta, botVis.getLastOrientation().thirdAngle);

            float angleDelta = (float) angleDifference(angle.thirdAngle, botVis.getLastOrientation().thirdAngle);
            float rotate = Range.clip(angleDelta / angleRamp, -1, 1) * maxPower;
            float forward = maxPower * Range.clip(-robotDelta.get(1)/targetRamp, -1, 1);
            float strafe = maxPower * Range.clip(robotDelta.get(0)/targetRamp, -1, 1);

            Log.d("DELTAS", String.format("rot: %.3f, fwd: %.3f, stf: %.3f,",
                    rotate, forward, strafe));

            bot.setDrive(forward + rotate - strafe,
                    forward - rotate + strafe,
                    forward + rotate + strafe,
                    forward - rotate - strafe);

            onTarget = (
                    (Math.abs(angleDelta) < angleDeadband)
                    && (Math.abs(robotDelta.get(1)) < targetDeadband)
                    && (Math.abs(robotDelta.get(0)) < targetDeadband)
            );
        } else {
            bot.setDrive(0,0,0,0);
        }

        return onTarget;
    }

    public void deactivate() {
        botVis.deactivate();
    }
}