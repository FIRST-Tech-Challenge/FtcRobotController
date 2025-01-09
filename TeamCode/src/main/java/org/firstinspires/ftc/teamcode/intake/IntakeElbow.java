package org.firstinspires.ftc.teamcode.intake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ServoConf;
import org.firstinspires.ftc.teamcode.outtake.OuttakeElbow;

import java.util.HashMap;
import java.util.Map;

public class IntakeElbow {

    enum Position {
        VERTICAL,
        OUTSIDE,
        INSIDE
    };

    Telemetry           logger;

    boolean             isReady;
    Position            position;
    Servo               servo;
    Map<String, Double> positions = new HashMap<>();

    public Position getPosition() { return position; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm) {

        logger = tm;

        String status = "";
        isReady = true;

        ServoConf pitch  = config.getServo("intake-elbow-pitch");

        if(pitch == null)  { isReady = false; }

        if(!isReady) { status = " CONF" + status; }
        else {

            servo  = hwm.tryGet(Servo.class, pitch.getName());

            if(servo == null) { isReady = false;  }

            if(!isReady) { status = " HW" + status; }
            else {
                if (pitch.getReverse()) {
                    servo.setDirection(Servo.Direction.REVERSE);
                }

                positions = pitch.getPositions();
            }

        }
        if(isReady) { logger.addLine("==>  IN EL : OK"); }
        else        { logger.addLine("==>  IN EL : KO : " + status); }

        this.setVertical();
    }
    public void setVertical() {

        if( positions.containsKey("vertical") && isReady) {

            servo.setPosition(positions.get("vertical"));
            position = Position.VERTICAL;

        }

    }

    public void setOutside() {

        if( positions.containsKey("outside") && isReady) {

            servo.setPosition(positions.get("outside"));

            position = Position.OUTSIDE;

        }

    }
    public void setInside() {

        if( positions.containsKey("inside") && isReady) {

            servo.setPosition(positions.get("inside"));

            position = Position.INSIDE;

        }

    }

}


