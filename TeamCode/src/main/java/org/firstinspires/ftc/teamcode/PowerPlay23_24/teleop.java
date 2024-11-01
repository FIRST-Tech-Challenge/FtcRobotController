package org. firstinspires. ftc. teamcode. PowerPlay23_24;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;

import org.firstinspires.ftc.teamcode.TemplateJanx;


@TeleOp(name = "DriverControl")
public class teleop extends OpMode {
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private Servo clawLeft;
    private Servo clawRight;
    private DcMotorEx nodder;
    private DcMotorEx extender;
    private DcMotorEx rotator;
    private DcMotorEx screwRight;
    private DcMotorEx screwLeft;
    int target = 0;
    int y = 0;

    @Override
    public void init() {
        TemplateJanx janx = new TemplateJanx(hardwareMap);
        janx.wheelInit("frontRight", "backRight", "backLeft", "frontLeft");
        frontLeft = janx.fl;
        frontRight = janx.fr;
        backRight = janx.br;
        backLeft = janx.bl;
        janx.armInit("armExtension", "arm rotations", "ScrewLeft", "ScrewRight");
        extender = janx.ext;
        rotator = janx.turn;
        screwRight = janx.sr;
        screwLeft = janx.sl;

        janx.clawInit("clawLeft", "clawRight", "nodder");
        clawLeft = janx.lc;
        clawRight = janx.rc;
        nodder = janx.nod;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator.setTargetPositionTolerance(2);
        rotator.setVelocityPIDFCoefficients(10,1,0,100);
        rotator.setPositionPIDFCoefficients(5);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        nodder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        nodder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nodder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nodder.setTargetPositionTolerance(2);
        nodder.setVelocityPIDFCoefficients(4.96,0.496,0,49.6);
        nodder.setPositionPIDFCoefficients(5);
//        nodder.setTargetPosition(0);
//        nodder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {
        mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("nod current",nodder.getCurrentPosition());
        telemetry.addData("nod target",nodder.getTargetPosition());
        telemetry.addData("power",nodder.getPower());
        telemetry.update();
        claw();
        arm();
        lift();
        preset();
        telemetry.update();
    }

    private void mecanum(double LSY, double LSX, double RSX) {
        int Speed = 1600;
        double lx = Math.pow(LSX, 3);
        double ly = -(Math.pow(LSY, 3));
        double rx = Math.pow(RSX, 3);
        //is RSX backwards? I may need to fix the canva
        if (LSX != 0 || LSY != 0 || RSX != 0) {
            frontRight.setVelocity(Speed * (clip((ly) - lx, -1, 1) - rx));
            frontLeft.setVelocity(Speed * (clip((ly) + lx, -1, 1) + rx));
            backRight.setVelocity(Speed * (clip((ly) + lx, -1, 1) - rx));
            backLeft.setVelocity(Speed * (clip((ly) - lx, -1, 1) + rx));
        } else {
            frontLeft.setVelocity(0);
            backLeft.setVelocity(0);
            frontRight.setVelocity(0);
            backRight.setVelocity(0);
        }
    }
    private void arm() {
        //rotator.setPower(Math.pow(gamepad2.right_stick_y,3));
        if(!gamepad2.a||!gamepad2.y||!gamepad2.x||!gamepad2.b) {
            if (gamepad2.right_stick_y > 0) {
                /* goes left? */
                target += 6;
                rotator.setVelocity(800);
            } else if (gamepad2.right_stick_y < 0) {
                /* goes right? */
                target -= 6;
                rotator.setVelocity(800);
            }
            else{
                rotator.setVelocity(0);
            }
//            if (target < -10) {
//                target = 0;
//                rotator.setVelocity(0);
//            }
            if (gamepad2.right_stick_y == 0){
                target = rotator.getCurrentPosition();
            }
            rotator.setTargetPosition(target);
        }
    }
    private void claw() {
        /**the claw nodder**/
        if(gamepad2.left_stick_y>0){
            y+=3;
        }
        if(gamepad2.left_stick_y<0){
            y-=3;
        }
        nodder.setPower(1);
        nodder.setTargetPosition(y);
        nodder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad2.right_bumper){
                /**close**/
                clawLeft.setPosition(-1);
            }
            else if(gamepad2.right_trigger!=0){
                /**open**/
                clawLeft.setPosition(1);
            }

            if (gamepad2.left_bumper) {
                /**close**/
//                clawRight.setPosition(clawRight.getPosition()+p);
                clawRight.setPosition(1);
            }
            else if(gamepad2.left_trigger!=0){
                /**open**/
//                clawRight.setPosition(clawRight.getPosition()-p);
                clawRight.setPosition(-.8);
            }
    }
    private void lift() {
        if (gamepad1.y) {
            screwLeft.setPower(1);
            screwRight.setPower(1);
        }
        else if (gamepad1.a) {
            screwLeft.setPower(-1);
            screwRight.setPower(-1);
        }
        else{
            screwLeft.setPower(0);
            screwRight.setPower(0);
        }
    }
    public void preset(){
        if(gamepad1.b){
            y = 0;
            target = 328;
        }
        else{
            if(gamepad2.x){
                //ground
                y = -130;
                target = 0;
            }
            else if(gamepad2.y){
                //up
                y = -150;
                target = 479;
            }
            else if(gamepad2.a){
                //down
                y = -175;
                target = 550;
            }
            else if(gamepad2.b){
                //forward
                y = -92;
                target = 66;
            }

        }
        nodder.setTargetPosition(y);
        rotator.setTargetPosition(target);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        nodder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//            if (gamepad2.left_stick_y > 0) {
//                y+=5;
////                nodder.setVelocity(200);
//            } else if (gamepad2.left_stick_y < 0) {
//                y-=5;
////                nodder.setVelocity(100);
//            }
////            else{
////                nodder.setVelocity(0);
////            }
//            nodder.setPower(1);
//            nodder.setTargetPosition(y);
//            nodder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

}

