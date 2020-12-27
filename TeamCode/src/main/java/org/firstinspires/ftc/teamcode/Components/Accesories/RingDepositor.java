/**
 * This class includes all the necessary functions
 * for the ring depositor to work in both Autonomous & Teleop.
 * For example goToPosition
 *
 *
 * @author Aiden Ma
 * @date 11.15.20
 * @versin 1.0
 * @status working in Teleop
 */

package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RingDepositor {

    public enum Position {
        REST, FLOOR
    }

    private LinearOpMode op = null;

    //TODO <owner> : Do we need to store the reference to hardware map here?
    private HardwareMap hardwareMap = null;

    private DcMotor ringDepositorMotor = null;

    //TODO <owner> : What is the right access modifier here?
    Servo ringClampServo = null;

    private final int ticksForREST = 0;
    private final int ticksForFLOOR = 908 ;
    private final double ringDepositorSpeed = 0.25;

    public RingDepositor(LinearOpMode opMode) {
        //setting the opmode & hardwareMap
        this.op = opMode;
        hardwareMap = op.hardwareMap;

        ringDepositorMotor = (DcMotor) opMode.hardwareMap.get("RingDepositorMotor");
        ringClampServo = hardwareMap.servo.get("RingClampServo");
        ringDepositorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ringDepositorMotor.setDirection(DcMotor.Direction.FORWARD);
        ringDepositorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringDepositorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ringClampServo.setPosition(0.0);
    }

    //tells motor to go to a specified position based on ticks(i)
    public void goToPosition(Position p) {
        int i = 0;
        if (p == Position.REST) {
            i = ticksForREST;
        } else if (p == Position.FLOOR) {
            i = ticksForFLOOR;
        } else {
            op.telemetry.addData("IQ Lvl", "0.00");
            op.telemetry.update();
            op.sleep(2000);
        }

        ringDepositorMotor.setTargetPosition(i);
        ringDepositorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ringDepositorMotor.setPower(ringDepositorSpeed);
        op.sleep(100);
        op.telemetry.addData("Ring Depositor", "Position:" + ringDepositorMotor.getCurrentPosition() + "-->" + i);
        op.telemetry.update();
        op.sleep(500);
        while (op.opModeIsActive() && ringDepositorMotor.isBusy()) {
            op.telemetry.addData("Ring Depositor: ", ringDepositorMotor.getCurrentPosition() + " busy=" + ringDepositorMotor.isBusy());
            op.telemetry.update();
            op.idle();
        }
        //op.sleep(2000);
        //brake
        ringDepositorMotor.setPower(0);
        ringDepositorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //TODO: <owner> : Can we start the method name with a verb?
    public void smartDeposit(){
        ringDepositorMotor.setTargetPosition(ticksForFLOOR);
        ringDepositorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ringDepositorMotor.setPower(ringDepositorSpeed);
        op.sleep(1500);
        ringClampServo.setPosition(1.0);
        op.sleep(500);
        ringDepositorMotor.setTargetPosition(ticksForREST);
        ringDepositorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ringDepositorMotor.setPower(ringDepositorSpeed);
        op.sleep(2000);
        ringClampServo.setPosition(0.0);
    }

    public void printCurrentLocation(){
        op.telemetry.addData("Wobble Goal ", "Current Position:" + ringDepositorMotor.getCurrentPosition());
        op.telemetry.update();
//        op.sleep(2000);
    }

    //stops the motor
    public void stop() {
        ringDepositorMotor.setPower(0);
    }

    // moves te ring clamp servo
    public void moveRingClamp(boolean direction){
        if (direction){
            ringClampServo.setPosition(1.0);
        } else {
            ringClampServo.setPosition(0.0);
        }
        op.telemetry.addData("Ring Clamp Position: ", direction);
        op.telemetry.update();
        op.sleep(500);
    }

}
