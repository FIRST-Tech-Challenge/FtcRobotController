package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class RFSlides extends RFMotor{
    /*init: reset, motorname, need encoder, zeroPowerBehavior brake
     * slideToPosition while loop, getEncoderPosition, setVelocity, DO NOT set_target_position
     * getSlidePosition*/

    private RFMotor extensionMotor;

    private final double MAX_EXTENSION_TICKS = 3300;
    private final double MIN_EXTENSION_TICKS = 15;

    LinearOpMode op;

    public RFSlides(String motorName, DcMotorSimple.Direction motorDirection, LinearOpMode opMode, boolean resetPos, DcMotor.ZeroPowerBehavior zeroBehavior) {
        super(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior);

        extensionMotor = new RFMotor(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior);

        op = opMode;

    }
    public void slidesToPosition(double targetPosition) {
        if (targetPosition > MAX_EXTENSION_TICKS) {
            targetPosition = MAX_EXTENSION_TICKS - 5;
        }
        double distance = targetPosition-getCurrentPosition();

        while (Math.abs(distance) > 20) {

            distance = targetPosition-getCurrentPosition();
            op.telemetry.addData("current position:", getCurrentPosition());
            setVelocity(distance/abs(distance) * 4 * (abs(distance) + 100));
            op.telemetry.addData("distance", distance);
            op.telemetry.update();
        }
        setVelocity(0);
    }

    public void slidesToPositionTeleop (double targetPosition) {
        if(targetPosition>MAX_EXTENSION_TICKS){
            targetPosition=MAX_EXTENSION_TICKS-5;
        }
        double distance = targetPosition-getCurrentPosition();
        if(abs(distance)<50){
            setVelocity(0);
        }
        else {
            setVelocity(distance/abs(distance) * 4 * (abs(distance) + 500));
        }
    }

    public int getCurrentPosition() {
        return extensionMotor.getCurrentPosition();
    }

    public void setVelocity(double velocity) {
        extensionMotor.setVelocity(velocity);
    }
}