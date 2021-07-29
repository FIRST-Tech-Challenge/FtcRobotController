package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;


    //Summary of controls
    //Drivetrain
    //vertical movement = the left stick on controller 1(moving on the y-axis)
    //Strafing = the left stick on controller 1(moving on the x-axis)
    //Rotating = the right stick on controller 1(moving on the x-axis)
    //press up on the dpad to raise the power by 0.1 and down to lower it by 0.1 (on controller 1, default power=0.6)

    //Wobble
    //grabber = pressing a on controller 2(this will close the arm to open them press a again)
    //lifter = press up on the dpad to raise the lifter and down to lower it (on controller 2)
    //rotator =

    //shooter
    //shooter = pressing x on controller 2(this will turn the shooter on, press x again to turn it off)
    //loader = pressing y on controller 2(this will push the ring, press y again to let another ring in)

    //intake
    //intake = pressing b on controller 2(this will turn the intake on, press b again to turn it off)
    //conveyor =

    //occupied controls
    //Controller 1
    //left stick x-axis
    //left stick x-axis
    //right stick y-axis
    //dpad up
    //dpad down

    //Controller 2
    //a
    //b
    //x
    //y
    //dpad up
    //dpad down






    @TeleOp(name = "Scrimmage TeleOp", group = "")
    public class UltimateGoalTeleOp extends LinearOpMode {

        private static int BUTTON_DELAY = 250;

        private static DecimalFormat df2 = new DecimalFormat("#.##");
        private DcMotor frontLeft;
        private DcMotor backLeft;
        private DcMotor frontRight;
        private DcMotor backRight;
        private Servo grabber;
        private Servo rotator;
        private DcMotor lifter;
        private DcMotor intake;
        private DcMotorEx shooter;
        private Servo loader;
        private DcMotor conveyor;
        public static double FORWARD = 0.5;
        public static double BACK = 0.65;
        public static double WOBBLE_OUT = 0;
        public static double WOBBLE_IN = 1;




        /**
         * This function is executed when this Op Mode is selected from the Driver Station.
         */
        @Override
        public void runOpMode() {
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backRight = hardwareMap.dcMotor.get("backRight");
            grabber = hardwareMap.servo.get("grabber");
            rotator = hardwareMap.servo.get("rotator");
            lifter = hardwareMap.dcMotor.get("lifter");
            intake = hardwareMap.dcMotor.get("intake");
            conveyor = hardwareMap.dcMotor.get("conveyor");
            shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            loader = hardwareMap.servo.get("loader");
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            boolean isBPressed = false;
            boolean isAPressed = false;
            double openWobble = 0.3;
            double closeWobble = 1;
            ElapsedTime timeSinceLastPress = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            waitForStart();
            grabber.setPosition(openWobble);
            rotator.setPosition(WOBBLE_IN);
            loader.setPosition(BACK);
            int servoDelay = 200;
            int shooterVelocity = 1600;


            if (opModeIsActive()) {
                double wheelsPowerFactor = 0.6;
                double shooterPowerFactor = 1;
                timeSinceLastPress.reset();



                while (opModeIsActive()) {
                    //Drivetrain
                    double output_x = Math.pow(gamepad1.left_stick_x, 3);
                    double output_y = Math.pow(gamepad1.left_stick_y, 3);
                    double output_xRight = Math.pow(gamepad1.right_stick_x, 3);
                    double drive = -output_y* wheelsPowerFactor;//vertical movement = the left stick on controller one(moving on the y-axis)
                    double strafe = -(output_x * wheelsPowerFactor);//Strafing = the left stick on controller 1(moving on the x-axis)
                    double rotate = output_xRight * wheelsPowerFactor;//Rotating = the right stick on controller 1(moving on the x-axis)
                    double lift = gamepad2.left_stick_y / 2;
                    telemetry.addData("Rotate Value", gamepad1.right_stick_x * wheelsPowerFactor);
                    frontLeft.setPower(drive - (strafe - rotate));
                    backLeft.setPower(drive + strafe + rotate);
                    frontRight.setPower(-(drive + (strafe - rotate)));
                    backRight.setPower(-(drive - (strafe + rotate)));
                    lifter.setPower(lift);

                    //double v = ((-1.0f/2.0f)*gamepad2.left_stick_y)+(1.0f/2.0f);
                    //loader.setPosition(v);
                    //telemetry.addData("servo", v);
                    if (gamepad1.dpad_up && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){//press up on the dpad to raise the power by 0.1 and down to lower it by 0.1 (on controller 1, default power=0.6)
                        if (wheelsPowerFactor < 1) {
                            wheelsPowerFactor += 0.1;
                            timeSinceLastPress.reset();
                        }

                    }

                    if (gamepad1.dpad_down && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){
                        if (wheelsPowerFactor > 0)
                            wheelsPowerFactor -= 0.1;
                        timeSinceLastPress.reset();

                    }

                    //wobble
                    if (gamepad2.a && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){//grabber = pressing a on controller 2(this will close the arm to open them press a again)
                        if (grabber.getPosition() == closeWobble){
                            grabber.setPosition(openWobble);
                            timeSinceLastPress.reset();
                        }
                        else {
                            grabber.setPosition(closeWobble);
                            timeSinceLastPress.reset();
                        }
                    }

                    // rotator
                    if (gamepad2.b && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){//grabber = pressing a on controller 2(this will close the arm to open them press a again)
                        if (rotator.getPosition() != WOBBLE_OUT){
                            rotator.setPosition(WOBBLE_OUT);
                            timeSinceLastPress.reset();
                        }
                        else {
                            rotator.setPosition(WOBBLE_IN);
                            timeSinceLastPress.reset();
                        }
                    }

                    // shooter
                    if (gamepad2.dpad_up && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){//press up on the dpad to raise the power by 0.1 and down to lower it by 0.1 (on controller 1, default power=0.6)
                        if (servoDelay < 1000) {
                            servoDelay += 50;
                            timeSinceLastPress.reset();
                        }
                    }

                    if (gamepad2.dpad_down && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){
                        if (servoDelay > 0)
                            servoDelay -= 50;
                        timeSinceLastPress.reset();
                    }

                    if (gamepad2.x && timeSinceLastPress.milliseconds() >= BUTTON_DELAY) {//press up on the dpad to raise the power by 0.1 and down to lower it by 0.1 (on controller 1, default power=0.6)
                        if (shooter.getPower() > 0) {
                            shooter.setVelocity(0);
                            timeSinceLastPress.reset();
                        }
                        else {
                            shooter.setVelocity(shooterVelocity);//2268;90%power 2394;94% power
                            timeSinceLastPress.reset();
                        }
                    }
                    if (gamepad1.y && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){//press up on the dpad to raise the power by 0.1 and down to lower it by 0.1 (on controller 1, default power=0.6)
                        if (shooterVelocity < 2500) {
                            shooterVelocity += 100;
                            shooter.setVelocity(shooterVelocity);
                            timeSinceLastPress.reset();
                        }
                    }

                    if (gamepad1.a && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){
                        if (shooterVelocity > 0)
                            shooterVelocity -= 100;
                        shooter.setVelocity(shooterVelocity);
                        timeSinceLastPress.reset();
                    }



                    //shooter


                    if (gamepad1.right_bumper && timeSinceLastPress.milliseconds() >= BUTTON_DELAY) {//loader = pressing y on controller 2(this will push the ring, press y again to let another ring in)
                        loader.setPosition(FORWARD);
                        sleep(servoDelay);
                        loader.setPosition(BACK);
                        sleep(servoDelay);
                        timeSinceLastPress.reset();
                    }
                    if (gamepad1.x && timeSinceLastPress.milliseconds() >= BUTTON_DELAY) {//loader = pressing y on controller 2(this will push the ring, press y again to let another ring in)
                        loader.setPosition(FORWARD);
                        sleep(servoDelay);
                        loader.setPosition(BACK);
                        sleep(servoDelay);
                        timeSinceLastPress.reset();
                    }

                    if (gamepad1.left_bumper && timeSinceLastPress.milliseconds() >= BUTTON_DELAY) {
                        for (int l=3; l>0; l--){
                            loader.setPosition(FORWARD);
                            sleep(servoDelay);
                            loader.setPosition(BACK);
                            sleep(servoDelay);
                        }
                        timeSinceLastPress.reset();;
                    }
                    if (gamepad2.y && timeSinceLastPress.milliseconds() >= BUTTON_DELAY) {
                        if (grabber.getPosition() > 0){
                            grabber.setPosition(0);
                            timeSinceLastPress.reset();
                        }
                    }


                    //intake

                    if (gamepad2.right_bumper && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){//intake = pressing b on controller 2(this will turn the intake on, press b again to turn it off)
                        if (intake.getPower() <= 0){
                            intake.setPower(1);
                            conveyor.setPower(1);
                            timeSinceLastPress.reset();
                        }
                        else {
                            intake.setPower(0);
                            conveyor.setPower(0);
                            timeSinceLastPress.reset();
                        }

                    }
                    if (gamepad2.left_bumper && timeSinceLastPress.milliseconds() >= BUTTON_DELAY){
                        if (intake.getPower() >= 0){
                            intake.setPower(-1);
                            conveyor.setPower(-1);
                            timeSinceLastPress.reset();
                        }
                        else {
                            intake.setPower(0);
                            conveyor.setPower(0);
                            timeSinceLastPress.reset();
                        }
                    }




                    telemetry.addData("change number", 1.1);
                    telemetry.addData("Drive Power:", drive);
                    telemetry.addData("Wheel power factor:", df2.format(wheelsPowerFactor));
                    telemetry.addData("Servo Delay:",  df2.format(servoDelay));
                    telemetry.addData("Actual Shooter Velocity: ", df2.format(shooter.getVelocity()));
                    telemetry.addData("Set Shooter Velocity:    ", df2.format(shooterVelocity));
                    telemetry.update();
                }
            }
        }
    }


