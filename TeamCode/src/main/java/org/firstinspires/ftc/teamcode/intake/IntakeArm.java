package org.firstinspires.ftc.teamcode.intake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ServoConf;

import java.util.HashMap;
import java.util.Map;

public class IntakeArm {

    enum Position {
        TRANSFER,
        OVER_SUBMERSIBLE,
        LOOKING,
        GRABBING
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

        ServoConf leftPitch  = config.getServo("intake-arm-left-pitch");
        ServoConf rightPitch = config.getServo("intake-arm-right-pitch");

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

        this.setTransfer();

    }

    public void setTransfer() {

        if( positions.containsKey("transfer") && isReady) {

            leftServo.setPosition(positions.get("transfer").get("left"));
            rightServo.setPosition(positions.get("transfer").get("right"));

            leftServo.getController().pwmEnable();
            rightServo.getController().pwmEnable();

            position = Position.TRANSFER;

        }

    }

    public void setOverSubmersible() {

        if( positions.containsKey("overSub") && isReady) {

            leftServo.setPosition(positions.get("overSub").get("left"));
            rightServo.setPosition(positions.get("overSub").get("right"));

            leftServo.getController().pwmEnable();
            rightServo.getController().pwmEnable();

            position = Position.OVER_SUBMERSIBLE;

        }

    }

    public void setLooking() {

        if( positions.containsKey("look") && isReady) {

            leftServo.setPosition(positions.get("look").get("left"));
            rightServo.setPosition(positions.get("look").get("right"));

            leftServo.getController().pwmEnable();
            rightServo.getController().pwmEnable();

            position = Position.LOOKING;

        }

    }

    public void setGrabbing() {

        if( positions.containsKey("grab") && isReady) {

            leftServo.setPosition(positions.get("grab").get("left"));
            rightServo.setPosition(positions.get("grab").get("right"));

            leftServo.getController().pwmEnable();
            rightServo.getController().pwmEnable();

            position = Position.GRABBING;

        }

    }

    public void moveUp() {
        if(position == Position.GRABBING)              { this.setLooking();         }
        else if(position == Position.LOOKING)          { this.setOverSubmersible(); }
        else if(position == Position.OVER_SUBMERSIBLE) { this.setTransfer();        }
    }

    public void moveDown() {
        if(position == Position.LOOKING)               { this.setGrabbing();        }
        else if(position == Position.OVER_SUBMERSIBLE) { this.setLooking();         }
        else if(position == Position.TRANSFER)         { this.setOverSubmersible(); }
    }

}


