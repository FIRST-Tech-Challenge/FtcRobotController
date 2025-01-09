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

public class OuttakeClaw {

    enum Position {
        OPEN,
        CLOSED
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

        ServoConf roll  = config.getServo("outtake-claw");

        if(roll == null)  { isReady = false; }

        if(!isReady) { status = " CONF" + status; }
        else {

            servo  = hwm.tryGet(Servo.class, roll.getName());

            if(servo == null) { isReady = false;  }

            if(!isReady) { status = " HW" + status; }
            else {
                if (roll.getReverse()) {
                    servo.setDirection(Servo.Direction.REVERSE);
                }

                positions = roll.getPositions();
            }
        }
        if(isReady) { logger.addLine("==>  IN CL : OK"); }
        else        { logger.addLine("==>  IN CL : KO : " + status); }

        this.setOpen();

    }

    public void setOpen() {

        if( positions.containsKey("open") && isReady) {

            servo.setPosition(positions.get("open"));
            position = Position.OPEN;

        }

    }

    public void setClosed() {

        if( positions.containsKey("closed") && isReady) {

            servo.setPosition(positions.get("closed"));
            position = Position.CLOSED;

        }

    }
    public void switchPosition() {

        if( position == Position.OPEN) { this.setClosed(); }
        else                           { this.setOpen();   }

    }

}


