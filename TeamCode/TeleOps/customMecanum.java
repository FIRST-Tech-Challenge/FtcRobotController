package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;
import org.firstinspires.ftc.teamcode.myUtil.threads.teleOp.lineUp;

@SuppressWarnings("unused")
//@Disabled
@TeleOp(name="HIT THIS BUTTON")
public class customMecanum extends OpMode {
//    double power1,power2,power3,power4;

    //Initial variables
    double deflator;
    boolean unpressR = false;
    boolean unpressL = false;
    boolean unpressB = false;
    boolean unpressY = false;
    boolean unpressS = false;
    boolean unpressS2 = false;
    boolean angled = true;
    double twist = 0;
    double twistMulti = 2;
    lineUp lineUp;
    Hardware r = new Hardware();

    @Override
    public void init() {
        r.initRobot(this);
//        lineUp = new lineUp(this,r);
    }

    @Override
    public void loop() {

//        telemetry.addData("x",pos.x);
//        telemetry.addData("y",pos.y);
//        telemetry.update();
        double power1 = 0;
        double power2 = 0;
        double power3 = 0;
        double power4 = 0;

        //Allows the user to shift the "forward" of the robot left or right
        if (gamepad1.dpad_left) {
            unpressL = true;
        }else if (unpressL) {
            twist += Math.toRadians(twistMulti);
            unpressL = false;
        }
        if (gamepad1.dpad_right) {
            unpressR = true;
        }else if (unpressR) {
            twist += -Math.toRadians(twistMulti);
            unpressR = false;
        }
        telemetry.addData("Twist", Math.toDegrees(twist));



        if (true){//!lineUp.isAlive()) {

            //Determines how much the robot has been turned since the beginning of the opMode
            double angle2;

                try {
//                    angle2 = r.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                    angle2 = 0;
                } catch (Exception e) {
                    angle2 = 0;
                }

            angle2 %= 2 * Math.PI;


        /*
            flm______frm
            |     +    |
            blm______brm
         */


            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 1.0 : gamepad1.left_bumper ? 0.3 : 0.7;
            //this first section creates the variables that will be used later
            //This is the mecanum drive code, it is weird
            //first we must translate the rectangular values of the joystick into polar coordinates;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            if (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) < 0.2) {
                x = 0;
                y = 0;
            }
            double angle = 0;

            if (y > 0 && x > 0)//quadrant 1
                angle = Math.atan(y / x);
            else {
                double angle1 = Math.toRadians(180) + Math.atan(y / x);
                if (y > 0 && x < 0)//quadrant 2
                    angle = angle1;
                else if (y < 0 && x < 0)//quadrant 3
                    angle = angle1;
                else if (y < 0 && x > 0)//quadrant 4
                    angle = Math.toRadians(360) + Math.atan(y / x);
            }

            if (y == 0 && x > 1) {
                angle = 0;
            }
            if (y > 0 && x == 0) {
                angle = Math.PI / 2;
            }
            if (y == 0 && x < 0) {
                angle = Math.PI;
            }
            if (y < 0 && x == 0) {
                angle = 3 * Math.PI / 2;
            }

            double velocity = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
            double rotation = gamepad1.right_stick_x;

            //equations taking the polar coordinates and turing them into motor powers
            double v1 = 0;
            double v2 = 0;
            if (angled) {
                 v1 = velocity * Math.cos(angle + (Math.PI / 4) - angle2 - twist);
                 v2 = velocity * Math.sin(angle + (Math.PI / 4) - angle2 - twist);
            }else{
                 v1 = velocity * Math.cos(angle + (Math.PI / 4));
                 v2 = velocity * Math.sin(angle + (Math.PI / 4));
            }


            power1 = v1 - rotation;
            power2 = v2 - rotation;
            power3 = v2 + rotation;
            power4 = v1 + rotation;

        }
        r.flm.setPower(power3 * deflator);
        r.frm.setPower(power4 * deflator);
        r.blm.setPower(power1 * deflator);
        r.brm.setPower(power2 * deflator);

        //Allows the robot to place itself in a specific location

        //Allows the user to disable the turning of the robot relative to the driver
        if (gamepad1.b){
            unpressB = true;
        }else if (unpressB){
            angled = !angled;
            unpressB = false;
        }

        //TODO REVERSE ALL WHEELS BUT BACK RIGHT
        //ITLL BE A START
        //DO YOU KNOW HOW?

        //Resets the robot "forward" position
        if (gamepad1.y){
            unpressY = true;
        }else if (unpressY){
            r.imu.initialize(r.parameters);
            unpressY = false;
        }
        if (gamepad2.right_bumper){
            unpressS = true;
        }else if (unpressS){
            unpressS = false;
        }
        if (gamepad2.left_bumper){
            unpressS2 = true;
        }else if (unpressS2){
            unpressS2 = false;
        }

        if(gamepad2.left_bumper) {
            r.linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
            r.linear_slide.setPower(0.3);
        }
        else if(gamepad2.right_bumper){
            r.linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);
            r.linear_slide.setPower(0.3);
        }else{
            r.linear_slide.setPower(0);
        }

//        if(gamepad2.right_trigger > 0){
//            r.arm.setDirection(DcMotorSimple.Direction.REVERSE);
//            r.arm.setPower(gamepad2.right_trigger * 0.8);
//        }
//        else if(gamepad2.left_trigger > 0){
//            r.arm.setDirection(DcMotorSimple.Direction.FORWARD);
//            r.arm.setPower(gamepad2.left_trigger * 0.8);
//        }
//        else{
//            r.arm.setPower(0);
//        }

        if(gamepad2.left_trigger > 0){
            r.claw.setPosition(1);
        }else if(gamepad2.right_trigger > 0){
            r.claw.setPosition(0);
        }

        r.arm1.setPosition(Math.abs(gamepad2.left_stick_y - 1) * 0.5);


        if(gamepad2.right_stick_y > 0){
            r.arm.setDirection(DcMotorSimple.Direction.REVERSE);
            r.arm.setPower(Math.abs(gamepad2.right_stick_y * 0.7));
        }
        else if(gamepad2.right_stick_y < 0){
            r.arm.setDirection(DcMotorSimple.Direction.FORWARD);
            r.arm.setPower(Math.abs(gamepad2.right_stick_y * 0.7));
        }else{
            r.arm.setPower(0);
        }

        telemetry.update();


    }

}
