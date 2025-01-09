package org.firstinspires.ftc.teamcode.intake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.MotorConf;

public class IntakeSlides {

    Telemetry            logger;

    boolean              areReady;
    DcMotor              motor;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm) {

        logger = tm;

        String status = "";
        areReady = true;

        MotorConf slides = config.getMotor("intake-slides");
        if (slides == null) {
            status += " CONF";
            areReady = false;
        } else {
            motor = hwm.tryGet(DcMotor.class, slides.getName());
            if (motor == null) {
                status += " HW";
                areReady = false;
            } else {
                if (slides.getReverse()) {
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        if (areReady) { logger.addLine("==>  IN SL : OK"); }
        else {          logger.addLine("==>  IN SL : KO : " + status); }
    }

    public void extend(double Power)   {
        if(areReady) {
            motor.setPower(Power);
        }
    }

    public void stop() {
        if (areReady) {
            motor.setPower(0);
        }
    }

    public void rollback(double Power) {
        if(areReady) {
            motor.setPower(-Power);
        }
    }

}


