package org.firstinspires.ftc.teamcode.outtake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configurations includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorSingle;

public class OuttakeSlides {

    Telemetry            mLogger;

    boolean              mReady;

    MotorComponent       mMotor;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;
        String status = "";

        ConfMotor slides = config.getMotor("outtake-slides");
        if(slides == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (slides.shallMock()) { mMotor = new MotorMock("outtake-slides"); }
            else if (slides.getHw().size() == 1) { mMotor = new MotorSingle(slides, hwm, "outtake-slides", logger); }
            else if (slides.getHw().size() == 2) { mMotor = new MotorCoupled(slides, hwm, "outtake-slides", logger); }

            if (!mMotor.isReady()) { mReady = false; status += " HW";}
            else {
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
            }
        }

        // Log status
        if (mReady) { logger.addLine("==>  OUT SLD : OK"); }
        else        { logger.addLine("==>  OUT SLD : KO : " + status); }

    }

    public void extend(double Power)   {
        if(mReady) {
            mMotor.setPower(Power);
        }
    }

    public void stop() {
        if (mReady) {
            mMotor.setPower(0);
        }
    }

    public void rollback(double Power) {
        if(mReady) {
            mMotor.setPower(-Power);
        }
    }

}


