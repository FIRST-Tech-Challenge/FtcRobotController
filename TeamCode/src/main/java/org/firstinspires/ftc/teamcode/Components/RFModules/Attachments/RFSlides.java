package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretStates.SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Components.Turret.checker;
import static org.firstinspires.ftc.teamcode.Components.Turret.extendPosition;
import static org.firstinspires.ftc.teamcode.Robot.faked;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.StateMachine;

import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFSlides extends RFMotor{
    /*init: reset, motorname, need encoder, zeroPowerBehavior brake
     * slideToPosition while loop, getEncoderPosition, setVelocity, DO NOT set_target_position
     * getSlidePosition*/

    private RFMotor extensionMotor;

    private final double MAX_EXTENSION_TICKS;
    private final double MIN_EXTENSION_TICKS;
    private final double TICKS_PER_INCH = 100.0;

    LinearOpMode op;

    public RFSlides(String motorName, DcMotorSimple.Direction motorDirection, LinearOpMode opMode, boolean resetPos, double MAX_EXTENSION_TICKS_INPUT, double MIN_EXTENSION_TICKS_INPUT, DcMotor.ZeroPowerBehavior zeroBehavior, Logger log) {
        super(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior, log);

        extensionMotor = new RFMotor(motorName, motorDirection, opMode, RUN_USING_ENCODER, resetPos, zeroBehavior, log);

        MAX_EXTENSION_TICKS = MAX_EXTENSION_TICKS_INPUT;
        MIN_EXTENSION_TICKS = MIN_EXTENSION_TICKS_INPUT;
        op = opMode;

    }
    public void slidesToPosition(double targetPosition) {
        if (targetPosition > MAX_EXTENSION_TICKS) {
            targetPosition = MAX_EXTENSION_TICKS - 5;
        }

        if (targetPosition < MIN_EXTENSION_TICKS) {
            targetPosition = MIN_EXTENSION_TICKS + 5;
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

    public void TurretManualExtension (double turret_extension) { //pog
        if (checker.checkIf(SLIDES_EXTENDED)) {
            if (((extendPosition > MAX_EXTENSION_TICKS && turret_extension >0) || (extendPosition < MIN_EXTENSION_TICKS && turret_extension<0))) {
                extensionMotor.setPower(0);
                op.telemetry.addData("extreme", extendPosition);

            }
            else if(abs(turret_extension)<.2){
                extensionMotor.setPower((0));
                op.telemetry.addData("little", extendPosition);
            }
            else {
                extensionMotor.setPower((turret_extension));
                op.telemetry.addData("tick", "ext", "retr", extendPosition, turret_extension);

            }
        }
        op.telemetry.addData("turtext", extendPosition);

    }

    public void turretExtendo(double whereExtendo){ //pog
        if(whereExtendo>MAX_EXTENSION_TICKS){
            whereExtendo=MAX_EXTENSION_TICKS-5;
        }
        double distance = whereExtendo-extendPosition;
        if(abs(distance)<20){
            extensionMotor.setVelocity(0);
        }
        else {
            extensionMotor.setVelocity(distance/abs(distance)* 4 * (abs(distance) + 50));
        }
        if(abs(distance)<20&&extensionMotor.getVelocity()<100){
            extensionMotor.setVelocity(0);
            faked=true;
        }
        else{
            faked = false;
        }
    }

    public int getCurrentPosition() {
        return extensionMotor.getCurrentPosition();
    }

    public void setVelocity(double velocity) {
        extensionMotor.setVelocity(velocity);
    }
}