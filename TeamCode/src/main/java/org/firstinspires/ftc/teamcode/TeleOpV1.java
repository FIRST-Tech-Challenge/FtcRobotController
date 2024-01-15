package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@TeleOp(name="2024-01-05", group="Linear Opmode")
public class TeleOpV1 extends LinearOpMode {
    
    private boolean[] button = {false, false, false, false};
    // 0 = compass, 1 = angle lock, 2 = constrain lift, 3 = claw toggle
    private boolean[] toggle = {true, false, false, false};
    // 0 = compass, 1 = angle lock, 2 = constrain lift, 3 = claw toggle
    
    boolean calibrated = false;
    //int liftPos = 0;
    int liftZero = 0;
    int liftStart = 0;
    final int LIFT_MAX = 380;
    final int LIFT_MIN = -5;
    
    RobotHardware H = new RobotHardware();
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ////////////////////////////// Init //////////////////////////////
        
        MecanumWheelDriverV2 drive = new MecanumWheelDriverV2(H);
        ElapsedTime runtime = new ElapsedTime();
        ExecutorService pool = Executors.newFixedThreadPool(2);
        H.setXYEncoderEnable(true);
        H.init(hardwareMap, this);
        //H.tracker.setInitialPosition(108,6);
        pool.execute(H);
        //pool.execute(tracker);
        ////////////////////////////// Init Variables //////////////////////////////
        
        long loopStart = 0;
        long loopTime;
        
        double y = 0;
        double x = 0;
        double rotate = 0;
        double rotateScaled = 0;
        double radius = 0;
        double power = 0;
        double headingLock = H.heading;
        boolean allowStartHeadingLock = true;
        double angle = 0;
        double rotateRadius;
        long lastTurnTime = 0;

        double upPower = 0;
        double downPower = 0;
        double lupPower = 0;
        double ldownPower = 0;
        
        double agl_frwd = 0;
        double heading = 0;
        
        //H.LEDMode = 0;
        
        waitForStart();
        runtime.reset();
    
        //H.LED.turnAllOff();
        //H.LEDMode = 2;
        
        //pool.execute(tracker);
        
        while (opModeIsActive()) {
    
            ////////////////////////////// Set Variables //////////////////////////////
            loopTime = runtime.time(TimeUnit.MILLISECONDS) - loopStart;
            loopStart = runtime.time(TimeUnit.MILLISECONDS);
    
            x = powerFollow(x, game pad1.left_stick_y, (double) loopTime / 1000);
            y = powerFollow(y, -gamepad1.right_stick_x, (double) loopTime / 1000);
    
            //rotate += powerFollow(rotate, exponentialScaling(Range.clip(gamepad1.right_stick_x, -1, 1)));
            rotate = exponentialScaling(Range.clip(gamepad1.left_stick_x, -1, 1));
            power = exponentialScaling(Range.clip(Math.hypot(x, y), 0, 1));
            if (gamepad1.left_bumper) {
                rotate /= 2;
                power /= 2;
            }
            
            if (power > 0.05) angle = Math.toDegrees(Math.atan2(y, x)) + 90 + agl_frwd - heading;
            drive.StrafePowerMove(angle, power, 1);
            
            if (Math.abs(rotate) < 0.05) {
                if (lastTurnTime + 350 < runtime.time(TimeUnit.MILLISECONDS)) {
                    if (toggle[1] && Math.abs(drive.findDegOffset(H.heading, headingLock)) > 1.5) {
                        if (allowStartHeadingLock) {
                            drive.HeadingRotate(headingLock, 0.15, 1);
                            allowStartHeadingLock = false;
                        }
                    } else {
                        allowStartHeadingLock = true;
                    }
                } else {
                    headingLock = H.heading;
                }
            } else {
                lastTurnTime = runtime.time(TimeUnit.MILLISECONDS);
                headingLock = H.heading;
                drive.PowerRotate(rotate, 1);
            }
    
            drive.startActions();
            
            upPower = Range.clip(-gamepad2.left_stick_y + gamepad1.right_trigger + gamepad2.right_trigger, 0, 1);
            downPower += Range.clip(gamepad2.left_stick_y + gamepad1.left_trigger + gamepad2.left_trigger, 0, 1);
            
            setLiftPower(upPower, downPower);
            if (gamepad1.left_bumper) {
                ldownPower = 1;
                lupPower = 0;
            }

            if (gamepad1.right_bumper) {
                ldownPower = 0;
                lupPower = 1;
            }
            setClimbPower(lupPower, ldownPower);
            downPower = 0;
            ldownPower = 0;
            lupPower = 0;
            
            ////////////////////////////// Buttons //////////////////////////////
            
            toggleButton(gamepad1.back, 0); // compass
            
            if (toggle[0]) {
                
                heading = H.heading;
                
            } else {
                
                heading = agl_frwd;
                
            }
    
            toggleButton(gamepad1.y, 1);
            toggleButton(gamepad1.b, 2);
            toggleButton(gamepad1.a || gamepad1.x || gamepad1.right_bumper || gamepad2.a || gamepad2.right_bumper || gamepad2.x, 3);
            
//            H.setGrabber(toggle[3]);
//            if (toggle[3]) { // open
//                H.clawServo.setPosition(H.GRABBER_SERVO_OPEN);
//            } else { // closed
//                H.clawServo.setPosition(H.GRABBER_SERVO_CLOSED);
//            }
            
            if (gamepad1.start) {
                agl_frwd = heading;
            }
            
            if (gamepad1.dpad_down) {
                //H.LEDEnable = false;
            } else if (gamepad1.dpad_up) {
                //
                // H.LEDEnable = true;
            }
    
            telemetry.addData("heading", H.heading);
            //telemetry.addData("heading lock", drive.findDegOffset(H.heading, headingLock));
            //telemetry.addData("claw", H.clawServo.getPosition());
            telemetry.addData("pos","x: (%2f), y: (%2f)", RobotTracker.output[0], RobotTracker.output[1]);
            telemetry.addData("MB","0: (%2f), 1: (%2f), 2: (%2f), 3: (%2f)", H.range[0], H.range[1], H.range[2], H.range[3]);
            // telemetry.addData("UP","right: (%2f), forward: (%2f), left: (%2f), back: (%2f)", H.tracker.ultrasonicPosition[0], H.tracker.ultrasonicPosition[1], H.tracker.ultrasonicPosition[2], H.tracker.ultrasonicPosition[3]);
            telemetry.addData("wheel FL", H.driveMotor[0].getPower());
            telemetry.addData("wheel FR", H.driveMotor[1].getPower());
            telemetry.addData("wheel RL", H.driveMotor[2].getPower());
            telemetry.addData("wheel RR", H.driveMotor[3].getPower());
            telemetry.update();
            
        }
        
        pool.shutdown();
        
    }
    
    private void setLiftPower(double upPower, double downPower) {
    
        double expUpPower = exponentialScaling(upPower);
        double expDownPower = exponentialScaling(downPower);
    
        telemetry.addData("lift pos: ", H.liftPos);
    
        if (Math.abs(downPower) < 0.05 && Math.abs(upPower) < 0.05) {
            H.liftMotor.setPower(0);
            return;
        }
        if (toggle[2]) {
            H.liftMotor.setPower(expUpPower);
            return;
        }
        
        if (toggle[2]) {
            H.liftMotor.setPower( -expDownPower);
            return;
        }
        
        if (toggle[2]) {
            H.liftMotor.setPower(adaptivePowerRamping(LIFT_MAX - H.liftPos, expUpPower, LIFT_MAX - liftStart) - adaptivePowerRamping(LIFT_MIN - H.liftPos, expDownPower, LIFT_MIN - liftStart));
            return;
        }
        
        H.liftMotor.setPower(expUpPower - expDownPower);
    }

    private void setClimbPower(double upPower, double downPower) {

        double expUpPower = exponentialScaling(upPower);
        double expDownPower = exponentialScaling(downPower);

        telemetry.addData("lift pos: ", H.liftPos);

        if (Math.abs(downPower) < 0.05 && Math.abs(upPower) < 0.05) {
            H.climbMotor.setPower(0);
            return;
        }

        H.climbMotor.setPower(expUpPower - expDownPower);
    }
    
    private void toggleButton(boolean gamepadIn, int numb) {
        
        if (gamepadIn) {
            
            if (!button[numb]) {
                
                toggle[numb] = !toggle[numb];
                button[numb] = true;
                
            }
            
        } else {
            
            button[numb] = false;
            
        }
        
    }
    
    double exponentialScaling(double input) {
        if (Math.abs(input) > H.STICK_DEAD_ZONE) {
            return Math.signum(input) * ((Math.pow(H.EXP_BASE, Math.abs(input)) - 1) * (1 - H.INITIAL_VALUE) / (H.EXP_BASE - 1) + H.INITIAL_VALUE);
        } else {
            return 0;
        }
    }
    
    double powerFollow(double currentPower, double goalPower, double deltaTime) {
        final double maxAcceleration = ((1 - (double) H.liftPos/H.MAX_LIFT_POS) * (H.MAX_ACCELERATION_SHORT - H.MAX_ACCELERATION_TALL)) + H.MAX_ACCELERATION_TALL;
        double deltaPower = maxAcceleration * deltaTime / H.MAX_VELOCITY;
        // double deltaPower = deltaTime / H.MAX_VELOCITY;
        
        //telemetry.addData("max acceleration", maxAcceleration);
        
        if (Math.abs(goalPower - currentPower) >= deltaPower) {
            return currentPower + Math.signum(goalPower - currentPower) * deltaPower;
        } else {
            return goalPower;
        }
    }
    
    private double adaptivePowerRamping(double offset, double power, double initialOffset) {
        
        return Range.clip((Math.exp(0.1 * Math.abs(offset))-1)/(Math.abs(power) * Math.pow(Math.abs(initialOffset/12),3)), 0.1, Math.abs(power));
        
    }
    
}

/* Unused

class Controller {
 
    
    Gamepad gamepad1;
    Gamepad gamepad2;
    
    boolean twoController;
    
    double move_x;
    double move_y;
    double rotate;
    double freight_lift_up;
    double freight_lift_down;
    double freight_lift;
    double ramp_lift_up;
    double ramp_lift_down;
    double ramp_lift;
    
    Controller(Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        twoController = false;
        
        move_x = this.gamepad1.left_stick_x;
        move_y = this.gamepad1.left_stick_y;
    }
    
    Controller(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        twoController = true;
        
    }
}*/
