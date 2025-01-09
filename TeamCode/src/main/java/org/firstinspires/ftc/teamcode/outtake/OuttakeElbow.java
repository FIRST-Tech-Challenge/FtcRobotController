package org.firstinspires.ftc.teamcode.outtake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ServoConf;
import org.firstinspires.ftc.teamcode.intake.IntakeArm;

import java.util.HashMap;
import java.util.Map;

public class OuttakeElbow {

    enum Position {
        VERTICAL,
        OUTSIDE,
        INSIDE
    };

    Telemetry                        logger;

    boolean                          isReady;
    Position                         position;
    Servo                            leftServo;
    Servo                            rightServo;
    Map<String, Map<String, Double>> positions   = new HashMap<>();

    public Position getPosition() { return position; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm) {

        logger = tm;

        String status = "";
        isReady = true;

        ServoConf leftPitch  = config.getServo("outtake-elbow-left-pitch");
        ServoConf rightPitch = config.getServo("outtake-elbow-right-pitch");

        if(leftPitch == null)  { status += "L" ; isReady = false; }
        if(rightPitch == null) { status += "R" ; isReady = false; }

        if(!isReady) { status = " CONF " + status; }
        else {

            leftServo  = hwm.tryGet(Servo.class, leftPitch.getName());
            rightServo  = hwm.tryGet(Servo.class, rightPitch.getName());

            if(leftServo == null) { status += "L" ; isReady = false;  }
            if(rightServo == null) { status += "R" ; isReady = false;  }

            if(!isReady) { status = " HW " + status; }
            else {
                if (leftPitch.getReverse()) {
                    leftServo.setDirection(Servo.Direction.REVERSE);
                }
                if (rightPitch.getReverse()) {
                    rightServo.setDirection(Servo.Direction.REVERSE);
                }

                Map<String, Double> lpositions = leftPitch.getPositions();
                Map<String, Double> rpositions = rightPitch.getPositions();
                lpositions.forEach((key, value) -> {
                    if(rpositions.containsKey(key)) {
                        HashMap<String,Double> temp = new HashMap<String,Double>();
                        temp.put("left", value);
                        temp.put("right", rpositions.get(key));
                        positions.put(key,temp);
                    }
                });


            }
        }
        if(isReady) { logger.addLine("==>  IN AR : OK"); }
        else        { logger.addLine("==>  IN AR : KO : " + status); }

        this.setVertical();

    }

    public void setVertical() {

        if( positions.containsKey("vertical") && isReady) {

            leftServo.setPosition(positions.get("vertical").get("left"));
            rightServo.setPosition(positions.get("vertical").get("right"));

            position = Position.VERTICAL;

        }

    }

    public void setOutside() {

        if( positions.containsKey("outside") && isReady) {

            leftServo.setPosition(positions.get("outside").get("left"));
            rightServo.setPosition(positions.get("outside").get("right"));

            position = Position.OUTSIDE;

        }

    }
    public void setInside() {

        if( positions.containsKey("inside") && isReady) {

            leftServo.setPosition(positions.get("inside").get("left"));
            rightServo.setPosition(positions.get("inside").get("right"));

            position = Position.INSIDE;

        }

    }

}


