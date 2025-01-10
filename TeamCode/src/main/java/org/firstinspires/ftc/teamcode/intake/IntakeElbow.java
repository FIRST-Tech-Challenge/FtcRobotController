package org.firstinspires.ftc.teamcode.intake;

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

public class IntakeElbow {

    enum Position {
        TRANSFER,
        OVER_SUBMERSIBLE,
        LOOKING,
        GRABBING
    };

    Telemetry               mLogger;

    boolean                 mReady;

    Position                mPosition;
    ServoComponent          mServo;
    Map<String, Double>     mPositions   = new LinkedHashMap<>();

    public Position getPosition() { return mPosition; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;

        String status = "";

        // Get configuration
        ConfServo pitch  = config.getServo("intake-elbow-pitch");
        if(pitch == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (pitch.shallMock()) { mServo = new ServoMock("intake-elbow-pitch"); }
            else if (pitch.getHw().size() == 1) { mServo = new ServoSingle(pitch, hwm, "intake-elbow-pitch", logger); }
            else if (pitch.getHw().size() == 2) { mServo = new ServoCoupled(pitch, hwm, "intake-elbow-pitch", logger); }

            mPositions = pitch.getPositions();
            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  IN ELB : OK"); }
        else        { logger.addLine("==>  IN ELB : KO : " + status); }

        // Initialize position
        this.setTransfer();

    }

    public void setTransfer() {

        if( mPositions.containsKey("transfer") && mReady) {
            mServo.setPosition(mPositions.get("transfer"));
            mPosition = Position.TRANSFER;
        }
    }

    public void setOverSubmersible() {

        if( mPositions.containsKey("overSub") && mReady) {
            mServo.setPosition(mPositions.get("overSub"));
            mPosition = Position.OVER_SUBMERSIBLE;
        }

    }

    public void setLooking() {

        if( mPositions.containsKey("look") && mReady) {
            mServo.setPosition(mPositions.get("look"));
            mPosition = Position.LOOKING;
        }

    }

    public void setGrabbing() {

        if( mPositions.containsKey("grab") && mReady) {
            mServo.setPosition(mPositions.get("grab"));
            mPosition = Position.GRABBING;
        }

    }

    public void moveUp() {
        if(mPosition == Position.GRABBING)              { this.setLooking();         }
        else if(mPosition == Position.LOOKING)          { this.setOverSubmersible(); }
        else if(mPosition == Position.OVER_SUBMERSIBLE) { this.setTransfer();        }
    }

    public void moveDown() {
        if(mPosition == Position.LOOKING)               { this.setGrabbing();        }
        else if(mPosition == Position.OVER_SUBMERSIBLE) { this.setLooking();         }
        else if(mPosition == Position.TRANSFER)         { this.setOverSubmersible(); }
    }

}


