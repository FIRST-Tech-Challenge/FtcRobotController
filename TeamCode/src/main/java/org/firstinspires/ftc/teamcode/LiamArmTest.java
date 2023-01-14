package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.motorSlide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="LiamArmTest", group="B")
public class LiamArmTest extends DriveMethods {
    DcMotor motorScissor30;
    DcMotor motorScissor60;
    int i = 0;
    int scissorHeight = 0;
    double clicksPerRot30 = 5281;
    double clicksPerRot60 = 2786;

    double targetClicks = 0;
    double target30 = 0;
    double target60 = 0; //target60 despite having half the value, needs to travel twice the distance, so they are in fact the same value :)
    double current30 = 0;
    double current60 = 0;
    double error30 = 0; //error of the 30rpm motor
    double error60 = 0; //error of the 60rpm motor
    double hingeScaleFactor = 0.979123; //This is 469mm/479mm (the length different between each 'arm')
    double power30 = 0;
    double power60 = 0;
    double currentAngle30 = 0;
    double currentAngle60 = 0;

    @Override
    public void runOpMode() {

        motorScissor30 = hardwareMap.get(DcMotor.class, "scissor30");
        motorScissor60 = hardwareMap.get(DcMotor.class, "scissor60");


        motorScissor60.setDirection(DcMotorSimple.Direction.REVERSE);

        motorScissor30.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorScissor60.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorScissor30.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorScissor60.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (opModeIsActive()) {
            if(gamepad2.dpad_up){
                scissorHeight++;
                sleep(200);
            }
            if(gamepad2.dpad_down){
                scissorHeight--;
                sleep(200);

            }

            targetClicks = (scissorHeight/360.0)*clicksPerRot30;

            target30 = targetClicks;
            target60 = targetClicks; //target60 despite having half the value, needs to travel twice the distance, so they are in fact the same value :)

            current30 = motorScissor30.getCurrentPosition();
            current60 = motorScissor60.getCurrentPosition();
            hingeScaleFactor = 0.979123; //This is 469mm/479mm (the length different between each 'arm')

            error30 = target30 - current30;
            error60 = target60 - current60;

            currentAngle30 = (current30/clicksPerRot30)*360;
            currentAngle60 = (current60/clicksPerRot60)*360;

            power30 = 0.45*(1 - (currentAngle30/90)) + error30/350;
            power60 = 0.33*(1- (currentAngle60/180)) + error60/600;

            motorScissor30.setPower(power30);
            motorScissor60.setPower(power60);

            telemetry.addLine("Scissor Height: " + scissorHeight);
            telemetry.addLine("Target Clicks: " + (scissorHeight/360)*clicksPerRot30);
            telemetry.addLine("30Motor Power: " + motorScissor30.getPower());
            telemetry.addLine("60Motor Power: " + motorScissor60.getPower());
            telemetry.addLine("target30: " + target30);
            telemetry.addLine("target60: " + target60);
            telemetry.addLine("30Position: " + motorScissor30.getCurrentPosition());
            telemetry.addLine("60Position: " + motorScissor60.getCurrentPosition());
            telemetry.addLine("error30: " + error30);
            telemetry.addLine("error60: " + error60);
            telemetry.addLine("power30: " + power30);
            telemetry.addLine("power60: " + power60);
            telemetry.addLine("currentAngle30: " + currentAngle30);
            telemetry.addLine("currentAngle60: " + currentAngle60);
            telemetry.update();



//            if(gamepad2.a) {
//                goToScissorHeight(scissorHeight);
//            }

            if(gamepad2.x){
                motorScissor60.setPower(0);
                motorScissor30.setPower(0);
            }
        }

    }


    // !!!!!! NO MORE THAN 1100 Clicks !!!!!!
    public void goToScissorHeight(double angle30) {
        /**
         * 60 rpm motor travels twice the distance and goes the opposite direction of the 30 rpm motor
         */
        //The "30" represents the 30 rpm base motor (first hinge)
        //The "60" represents the 60 rpm higher motor (second hinge)
        //Because they have different rpms by a factor of 2, 60rpm has half the clicks/rotation of the 30rpm


        int clicksPerRot30 = 5281;
        int clicksPerRot60 = 2786;

        int targetClicks = (int)((angle30/360)*clicksPerRot30);
        int target30 = targetClicks;
        int target60 = targetClicks; //target60 despite having half the value, needs to travel twice the distance, so they are in fact the same value :)
        double current30 = motorScissor30.getCurrentPosition();
        double current60 = motorScissor60.getCurrentPosition();
        double error30 = target30 - current30; //error of the 30rpm motor
        double error60 = target60 - current60; //error of the 60rpm motor
        double hingeScaleFactor = 0.979123; //This is 469mm/479mm (the length different between each 'arm')
        double power30 = 0;
        double power60 = 0;
        double currentAngle30 = 0;
        double currentAngle60 = 0;

        telemetry.addLine("30Position: " + current30);
        telemetry.addLine("60Position: " + current60);
        telemetry.addLine("error30: " + error30);
        telemetry.addLine("error60: " + error60);
        telemetry.addLine("power30: " + power30);
        telemetry.addLine("power60: " + power60);
        telemetry.addLine("angle30: " + currentAngle30);
        telemetry.addLine("angle60: " + currentAngle60);

        telemetry.update();

        sleep(500);

        while(Math.abs(error30) >= 5 || Math.abs(error60) >= 5){
            current30 = motorScissor30.getCurrentPosition();
            current60 = motorScissor60.getCurrentPosition(); //This value's resolution is half the resolution of the above value

            error30 = target30 - current30;
            error60 = target60 - current60;

            currentAngle30 = (current30/clicksPerRot30)*360;
            currentAngle60 = (current60/clicksPerRot60)*360;

            power30 = 0.45*(1 - (currentAngle30/90)) + error30/350;
            power60 = 0.33*(1- (currentAngle60/180)) + error60/600;

            motorScissor30.setPower(power30);

            motorScissor60.setPower(power60); //the 60 rpm is traveling twice the distance of the 30 rpm at all times


            telemetry.addLine("target30: " + target30);
            telemetry.addLine("target60: " + target60);
            telemetry.addLine("30Position: " + motorScissor30.getCurrentPosition());
            telemetry.addLine("60Position: " + motorScissor60.getCurrentPosition());
            telemetry.addLine("error30: " + error30);
            telemetry.addLine("error60: " + error60);
            telemetry.addLine("power30: " + power30);
            telemetry.addLine("power60: " + power60);
            telemetry.addLine("currentAngle30: " + currentAngle30);
            telemetry.addLine("currentAngle60: " + currentAngle60);

            telemetry.update();

//            if(currentAngle30 > 80 || currentAngle60 > 160) {
//                motorScissor60.setPower(0);
//                motorScissor30.setPower(0);
//                break;
//            }
            i++;
        }


        telemetry.addLine("You're outside the loop!");
        telemetry.addLine("power30: " + (power30-0.04));
        telemetry.addLine("power60: " + (power60-0.04));
        telemetry.update();
        motorScissor30.setPower(power30 - 0.04);
        motorScissor60.setPower(power60 - 0.04);


    }
}
//hi