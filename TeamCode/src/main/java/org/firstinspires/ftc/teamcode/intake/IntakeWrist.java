package org.firstinspires.ftc.teamcode.intake;

/* System includes */
import static java.lang.Math.max;
import static java.lang.Math.min;

import java.util.LinkedHashMap;
import java.util.Map;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.ServoMock;
import org.firstinspires.ftc.teamcode.components.ServoCoupled;
import org.firstinspires.ftc.teamcode.components.ServoSingle;

public class IntakeWrist {

    enum Position {
        CENTER,
        UNDEFINED
    };

    Telemetry           mLogger;

    boolean             mReady;
    Position            mPosition;
    double              mDeltaPosition = 0;
    ServoComponent      mServo;
    Map<String, Double> mPositions = new LinkedHashMap<>();

    public Position getPosition() { return mPosition; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;

        String status = "";

        // Get configuration
        ConfServo roll  = config.getServo("intake-wrist-roll");
        if(roll == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (roll.shallMock()) { mServo = new ServoMock("intake-wrist-roll"); }
            else if (roll.getHw().size() == 1) { mServo = new ServoSingle(roll, hwm, "intake-wrist-roll", logger); }
            else if (roll.getHw().size() == 2) { mServo = new ServoCoupled(roll, hwm, "intake-wrist-roll", logger); }

            mPositions = roll.getPositions();
            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  IN WRS : OK"); }
        else        { logger.addLine("==>  IN WRS : KO : " + status); }

        // Initialize position
        this.setCenter();
    }

    public void setCenter() {

        if( mPositions.containsKey("center") && mReady) {
            mServo.setPosition(mPositions.get("center"));
            mPosition = Position.CENTER;
            mDeltaPosition = 0;
        }

    }

    public void turn(double increment)
    {
        if( mPositions.containsKey("center") &&
                mPositions.containsKey("min") &&
                mPositions.containsKey("max") &&
                mReady) {

            mDeltaPosition += increment;
            double newPosition = mPositions.get("center") + mDeltaPosition;

            newPosition = max(newPosition, mPositions.get("min"));
            newPosition = min(newPosition, mPositions.get("max"));

            mServo.setPosition(newPosition);

            mPosition = Position.UNDEFINED;
        }
    }

}


