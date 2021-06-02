package org.firstinspires.ftc.team417_2020;

import org.firstinspires.ftc.team417_2020.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;
    private boolean openWobbleGoalGrabber = false;
    //true is in, false is out
    private boolean RingPusher = false;
    //true is up, false is down
    private boolean isRampUp = false;

    private Toggler slowMode = new Toggler();
    private Toggler reverseMode = new Toggler();
    private Toggler wobbleGoalGrabberToggler = new Toggler();
    private Toggler RingPusherToggler = new Toggler();
    private Toggler RampToggler = new Toggler();

    int wobbleGoalPosition = 0;

    double velocity = -0.9;
    //ElapsedTime time = new ElapsedTime();

    private long initialTime = 0;




    /**
     * Uses the mecanum drive function to move the robot | Right stick translate, Left stick rotate (gamepad1)
     */
    public void driveRobot()
    {
        isSlowMode = slowMode.toggle(gamepad1.right_bumper);
        isReverseMode = reverseMode.toggle(gamepad1.left_bumper);

        double y = -gamepad1.right_stick_y; // Y is negative above the Y axis
        double x = gamepad1.right_stick_x;
        double rotationalPower = gamepad1.left_stick_x;


        y *= 1 - (0.8 * gamepad1.right_trigger);
        x *= 1 - (0.8 * gamepad1.right_trigger);
        rotationalPower *= 1 - (0.8 * gamepad1.right_trigger);

        if (gamepad1.left_bumper) {
            y *= -1;
            x *= -1;
        }


        // todo check and test to see if we need filtering
        /*
        filterJoyStickInput.appendInput(x, y, pivotPower);

        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();
         */

        double drivePower = Math.hypot(x, y);
        double angle = Math.atan2(y, x);

        mecanumDrive(angle, drivePower, rotationalPower);
    }

    public void moveWobbleGoalArm() {

        // control arm position with controller 2 joystick
        wobbleGoalPosition += gamepad2.left_stick_y * 3;
        if (wobbleGoalPosition < -584) {
            wobbleGoalPosition = -584;
        }
        telemetry.addData("Wobble Goal Position", wobbleGoalPosition);
        runMotorToPosition(motorWobbleGoalArm, wobbleGoalPosition, gamepad2.left_stick_y);

        // controller 2 A
        // control wobble goal grabber servo with Toggler
        openWobbleGoalGrabber = wobbleGoalGrabberToggler.toggle(gamepad2.a);
        if (openWobbleGoalGrabber) {
            wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_OUT);
        } else {
            wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_IN);
        }
    }

    public void collector() {
        //get Ramp Toggle state and set servo to position
        isRampUp = RampToggler.toggle(gamepad2.y);
        if (isRampUp) {
            servoRamp.setPosition(RAMP_UP);
        } else {
            servoRamp.setPosition(RAMP_DOWN);
        }

        if (gamepad2.y && isRampUp) {
            initialTime = System.currentTimeMillis();
        }

        if (gamepad2.right_bumper && isRampUp) {
            RampToggler.toggle(false);
            RampToggler.toggle(true);
        }

        //set power to collector
        if (gamepad2.right_bumper) {
            motorCollection.setPower(0.8);
        } else if (gamepad2.right_trigger != 0) {
            motorCollection.setPower(-0.8);
        } else if (System.currentTimeMillis() < initialTime + 300 && isRampUp) {
            motorCollection.setPower(0.8);
        } else {
            motorCollection.setPower(0.0);
        }
    }

    public void setLauncherVelocity() throws InterruptedException {
        //get RingPusher Toggle state and set servo to position
        RingPusher = RingPusherToggler.toggle(gamepad2.x);

        if (gamepad2.x) {
            servoRingPusher.setPosition(RING_PUSHER_IN);
        } else {
            servoRingPusher.setPosition(RING_PUSHER_OUT);
        }


        if (gamepad2.b) {
            for (int i = 0; i < 3; i++) {
                servoRingPusher.setPosition(RING_PUSHER_IN);
                servoRingPusher.setPosition(RING_PUSHER_OUT);
            }
        }
        //adjust speed of launcher and set motor to speed
        if (gamepad2.dpad_up) {
            velocity += 0.1;
            sleep(200);
        } else if (gamepad2.dpad_down){
            velocity -= 0.1;
        }
        if (Math.abs(velocity) > 1) {
            velocity = Math.signum(velocity) * 1.0;
        }

        // setting speed
        if ((gamepad2.left_bumper && gamepad2.left_trigger != 0) || (servoRamp.getPosition() == RAMP_UP && gamepad2.left_trigger != 0)) {
            motorLauncher.setPower(-0.85);
        }
        else if (gamepad2.left_bumper || servoRamp.getPosition() == RAMP_UP) {
            motorLauncher.setPower(velocity);
        }
        else {
            motorLauncher.setPower(0.0);
        }
        telemetry.addData("set velocity: ", velocity);


    }

}
