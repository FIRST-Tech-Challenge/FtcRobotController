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
        TRANSFER,
        OVER_SUBMERSIBLE,
        LOOKING,
        GRABBING
    };

    Telemetry           logger;

    boolean             isReady;
    Position            position;
    Servo               servo;
    Map<String, Double> positions = new HashMap<>();

    public Position getPosition() { return position; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm) {

        logger          = tm;

        String status   = "";
        isReady         = true;

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

        this.setTransfer();
    }
    public void setTransfer() {

        if( positions.containsKey("transfer") && isReady) {

            servo.setPosition(positions.get("transfer"));
            position = Position.TRANSFER;

        }

    }

    public void setOverSubmersible() {

        if( positions.containsKey("overSub") && isReady) {

            servo.setPosition(positions.get("overSub"));
            position = Position.OVER_SUBMERSIBLE;

        }

    }

    public void setLooking() {

        if( positions.containsKey("look") && isReady) {

            servo.setPosition(positions.get("look"));
            position = Position.LOOKING;

        }

    }

    public void setGrabbing() {

        if( positions.containsKey("grab") && isReady) {

            servo.setPosition(positions.get("grab"));
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


