package org.firstinspires.ftc.teamcode;

/**
 * Created by Karim on 3/9/2018.
 */

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;


public class LEDSystem {

    public static boolean ENABLED = true;

    Servo movement;
//    RevBlinkinLedDriver ledDriver;

    public enum Color {
        RED(1418), BLUE(1425), GOLD(1595), STRESS(1079), CALM(1108), GAME_OVER(1208), PURPLE(1787), OFF(1200),
        LIGHT_CHEESE_RED(1345), LIGHT_CHEESE_BLUE(1355), LIGHT_CHEESE_GREEN(1365), PARTY_MODE(1085), PARTY_MODE_SMOOTH(1215), PARTY_MODE_RAINBOW(1275), SHOT(1095);
        Color(int pos) {
            this.pos = pos;
        }

        public final int pos;
    }

    public LEDSystem(Servo ledServo) {
        this.movement = ledServo;
    }

//    public LEDSystem(Servo ledServo, RevBlinkinLedDriver led) {
//        this(ledServo);
//        this.ledDriver = led;
//    }

    public void setColor(Color color) {
        if (ENABLED)
            movement.setPosition(servoNormalize(color.pos));
    }

//    public void setColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
//        if (ENABLED)
//            ledDriver.setPattern(pattern);
//    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
