package org.firstinspires.ftc.teamcode.outtake;

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
import org.firstinspires.ftc.teamcode.intake.IntakeWrist;

public class OuttakeWrist {

    public enum Position {
        CENTER,
        MIN,
        MAX,
        UNDEFINED
    };

    private static final Map<String, Position> sConfToPosition = Map.of(
            "center", Position.CENTER,
            "min", Position.MIN,
            "max", Position.MAX
    );

    public static final double sIncrementRatio = 0.01;

    Telemetry             mLogger;

    boolean               mReady;
    Position              mPosition;
    double                mDeltaPosition = 0;
    ServoComponent        mServo;
    Map<Position, Double> mPositions = new LinkedHashMap<>();

    public Position getPosition() { return mPosition; }

    public double   getServo() {
        double result = -1.0;
        if (mPositions.containsKey(Position.CENTER)) {
            result = mPositions.get(Position.CENTER) + mDeltaPosition;
        }
        return result;
    }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;

        String status = "";

        // Get configuration
        ConfServo roll  = config.getServo("outtake-wrist-roll");
        if(roll == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (roll.shallMock()) { mServo = new ServoMock("outtake-wrist-roll"); }
            else if (roll.getHw().size() == 1) { mServo = new ServoSingle(roll, hwm, "outtake-wrist-roll", logger); }
            else if (roll.getHw().size() == 2) { mServo = new ServoCoupled(roll, hwm, "outtake-wrist-roll", logger); }

            mPositions.clear();
            Map<String, Double> confPosition = roll.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }
            }

            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  OUT WRS : OK"); }
        else        { logger.addLine("==>  OUT WRS : KO : " + status); }

        // Initialize position
        this.setPosition(Position.CENTER);
    }

    public void setPosition(Position position) {

        if( mPositions.containsKey(position) && mReady) {
            mServo.setPosition(mPositions.get(position));
            mPosition = position;
            mDeltaPosition = 0;
        }
    }

    public void turn(double increment)
    {
        if( mPositions.containsKey(Position.CENTER) &&
                mPositions.containsKey(Position.MIN) &&
                mPositions.containsKey(Position.MAX) &&
                mReady) {

            mDeltaPosition += increment * sIncrementRatio;
            double newPosition = mPositions.get(Position.CENTER) + mDeltaPosition;

            newPosition = max(newPosition, mPositions.get(Position.MIN));
            newPosition = min(newPosition, mPositions.get(Position.MAX));

            mLogger.addLine("" + newPosition);
            mLogger.addLine("" + mDeltaPosition);

            mServo.setPosition(newPosition);

            mPosition = Position.UNDEFINED;
        }
    }

}


