package org.firstinspires.ftc.teamcode.outtake;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.MotorConf;


public class OuttakeSlides {

    Telemetry    logger;

    boolean      areReady;
    DcMotor      leftMotor;
    DcMotor      rightMotor;

    static final double  MAX_POWER = 1.0;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm) {

        logger = tm;

        String status = "";
        areReady = true;

        MotorConf leftSlides = config.getMotor("outtake-left-slides");
        MotorConf rightSlides = config.getMotor("outtake-right-slides");
        if (leftSlides == null) {
            status += "L";
            areReady = false;
        }
        if (rightSlides == null) {
            status += "R";
            areReady = false;
        }
        if (!areReady) {
            status = " CONF " + status;
        } else {
            leftMotor = hwm.tryGet(DcMotor.class, leftSlides.getName());
            rightMotor = hwm.tryGet(DcMotor.class, rightSlides.getName());
            if (leftMotor == null) {
                status += "L";
                areReady = false;
            }
            if (rightMotor == null) {
                status += "R";
                areReady = false;
            }
            if (!areReady) {
                status = " HW " + status;
            } else {
                if (leftSlides.getReverse()) {
                    leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                if (rightSlides.getReverse()) {
                    rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        if (areReady) {
            logger.addLine("==>  OUT SL : OK");
        } else {
            logger.addLine("==>  OUT SL : KO : " + status);
        }

    }

    public void extend(double Power)   {
        if(areReady) {
            rightMotor.setPower(Power);
            leftMotor.setPower(Power);
        }
    }

    public void stop()                 {
        if(areReady) {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
    }

    public void rollback(double Power) {
        if(areReady) {
            rightMotor.setPower(-Power);
            leftMotor.setPower(-Power);
        }
    }

}


