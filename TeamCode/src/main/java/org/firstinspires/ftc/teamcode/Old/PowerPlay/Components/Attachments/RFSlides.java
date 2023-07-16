package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc.StateMachine.RobotStates.SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.checker;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware.Turret.extendPosition;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.faked;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

import java.util.ArrayList;

public class RFSlides extends RFMotor{
    /*init: reset, motorname, need encoder, zeroPowerBehavior brake
     * slideToPosition while loop, getEncoderPosition, setVelocity, DO NOT set_target_position
     * getSlidePosition*/

    private RFMotor extensionMotor;

    private ArrayList<Double> coefs = new ArrayList();

    private double MAX_EXTENSION_TICKS = 1060;
    private double MIN_EXTENSION_TICKS = 20;


    public RFSlides(String motorName, DcMotor.RunMode runMode, boolean resetPos, ArrayList<Double> coefficients, double maxtick,
                    double mintick) {
        super(motorName, runMode, resetPos, coefficients, maxtick, mintick);

        extensionMotor = new RFMotor(motorName, runMode, resetPos, coefficients, maxtick, mintick);

    }

//    public void slidesToPosition(double targetPosition) {
//        if (targetPosition > MAX_EXTENSION_TICKS) {
//            targetPosition = MAX_EXTENSION_TICKS - 5;
//        }
//
//        if (targetPosition < MIN_EXTENSION_TICKS) {
//            targetPosition = MIN_EXTENSION_TICKS + 5;
//        }
//
//        double distance = targetPosition-getCurrentPosition();
//
//        while (Math.abs(distance) > 20) {
//
//            distance = targetPosition-getCurrentPosition();
//            op.telemetry.addData("current position:", getCurrentPosition());
//            setVelocity(distance/abs(distance) * 4 * (abs(distance) + 100));
//            op.telemetry.addData("distance", distance);
//            op.telemetry.update();
//        }
//        setVelocity(0);
//    }

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
        if (checker.getState(SLIDES_EXTENDED)) {
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