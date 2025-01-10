package org.firstinspires.ftc.teamcode.outtake;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class OuttakeClaw {

    enum Position {
        OPEN,
        CLOSED
    };

    Telemetry           mLogger;

    boolean             mReady;
    Position            mPosition;
    ServoComponent      mServo;
    Map<String, Double> mPositions = new LinkedHashMap<>();

    public Position getPosition() { return mPosition; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;

        String status = "";

        // Get configuration
        ConfServo move  = config.getServo("outtake-claw");
        if(move == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (move.shallMock()) { mServo = new ServoMock("outtake-claw"); }
            else if (move.getHw().size() == 1) { mServo = new ServoSingle(move, hwm, "outtake-claw", logger); }
            else if (move.getHw().size() == 2) { mServo = new ServoCoupled(move, hwm, "outtake-claw", logger); }

            mPositions = move.getPositions();
            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  OUT CLW : OK"); }
        else        { logger.addLine("==>  OUT CLW : KO : " + status); }

        // Initialize position
        this.setOpen();
    }

    public void setOpen() {

        if( mPositions.containsKey("open") && mReady) {
            mServo.setPosition(mPositions.get("open"));
            mPosition = Position.OPEN;
        }

    }

    public void setClosed() {

        if( mPositions.containsKey("closed") && mReady) {
            mServo.setPosition(mPositions.get("closed"));
            mPosition = Position.CLOSED;
        }

    }

    public void switchPosition() {
        if( mPosition == Position.OPEN) { this.setClosed(); }
        else                            { this.setOpen();   }
    }

}