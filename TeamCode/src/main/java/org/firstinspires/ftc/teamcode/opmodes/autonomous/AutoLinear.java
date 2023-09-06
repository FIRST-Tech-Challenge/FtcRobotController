package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.hardware.ArmRelease;
import org.firstinspires.ftc.teamcode.subsystems.hardware.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.subsystems.hardware.ImuDevice;
import org.firstinspires.ftc.teamcode.subsystems.hardware.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.hardware.Lights;
import org.firstinspires.ftc.teamcode.subsystems.hardware.MecanumDriveByGyro;
import org.firstinspires.ftc.teamcode.subsystems.hardware.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.subsystems.hardware.RobotDevices;

@Autonomous(
        name = "Autonomous Mode"
)
public class AutoLinear extends LinearOpMode {

    protected int current_stack_height=LiftClaw.STACK_TOP_PICKUP;

    protected RobotDevices robotDevices;
    protected LiftClaw _liftclaw;
    protected ArmRelease armRelease;

    protected MecanumDriveByGyro _move;


    protected Lights light;

    protected final double LEFT=-90;
    protected final double RIGHT=90;
    protected static final int LEFT_SIDE=1;
    protected static final int RIGHT_SIDE=2;

    protected ColorSensorDevice colorSensorDeviceLeft, colorSensorDeviceRight;


    public ColorSensorDevice getColorSensorDevice() {
        if (left)
            return colorSensorDeviceRight;
        else
            return colorSensorDeviceLeft;
    };

    public void strafeDirection(double distance) {
        if (left) {
            driveRight(distance);
        }
        else {
            driveLeft(distance);
        }
    }
    public void strafeAntiDirection(double distance) {
        if (left) {
            driveLeft(distance);
        }
        else {
            driveRight(distance);
        }
    };
    public double turnDirection() {
        if (left)
            return this.LEFT;
        else
            return this.RIGHT;
    }

    public int side() {
        if (left)
            return LEFT_SIDE;
        else
            return RIGHT_SIDE;
    };

    boolean blue = false;
    boolean left = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        getOptions();

        robotDevices = RobotDevices.getDevices(hardwareMap);

        light = new Lights(hardwareMap.dcMotor.get("LIGHTS")){
            @Override
            public void on() {
                if (blue)
                    blueOn();
                else
                    redOn();
            }
        };

        // set up MovementThread

        colorSensorDeviceLeft = new ColorSensorDevice(robotDevices.colorSensorLeft);
        colorSensorDeviceRight = new ColorSensorDevice(robotDevices.colorSensorRight);

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.motors = robotDevices.wheels;
        driveParameters.ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.telemetry = telemetry;
        _move = new MecanumDriveByGyro(driveParameters, new ImuDevice(robotDevices.imu));

        // setup LiftClaw
        _liftclaw = new LiftClaw(
                robotDevices.lift_motor,
                robotDevices.lift_servos,
                //robotDevices.pipe_guide,
                robotDevices.bottom_stop,
                robotDevices.post_sensor,
                telemetry,
                //new GamepadEmpty(),
                light
        );

        armRelease = robotDevices.arm_release;


        _move.resetHeading();


        waitForStart();

        light.on();

        // start moving the bot.

        // right and left seemed to function differently.
        if (left) {
            runLeft();
        }
        else {
            runRight();
        }
    }

    public void getOptions() {


        String first_query = "Press X for Blue or B for Red";
        String second_query = "Press X for Left and B for Right";
        String double_check = "Press Y to confirm and A to repick";
        boolean RVB = false;
        boolean LVR = false;
        Telemetry.Item Query = telemetry.addData("","");
        //wait for keypad press on gamepad1
        while (!(LVR && RVB)) {

            if (!RVB) {
                Query.setValue(first_query);
                while (true) {
                    if (gamepad1.x) {
                        blue = true;
                        break;
                    } else if (gamepad1.b) {
                        blue = false;
                        break;
                    }
                }
                Query.setValue(double_check);
                while (true) {
                    if (gamepad1.a) {
                        break;
                    }
                    if (gamepad1.y) {
                        RVB = true;
                        break;
                    }
                }
             }
            if (! LVR) {
                Query.setValue(second_query);
                while (true) {
                    if (gamepad1.x) {
                        left = true;
                        break;
                    } else if (gamepad1.b) {
                        left = false;
                        break;
                    }
                }
                Query.setValue(double_check);
                while (true) {
                    if (gamepad1.a) {
                        break;
                    }
                    if (gamepad1.y) {
                        LVR = true;
                        break;
                    }
                }
            }

            Query.setValue("So we are " +(blue?"Blue":"Red") + " - " +(left?"Left":"Right") + "? - Y to confirm, A to cancel!");
            while (true) {
                if (gamepad1.a) {
                    LVR=false;
                    RVB=false;
                    break;
                }
                if (gamepad1.y) {
                    break;
                }
            }

        }
    }
    public void runLeft() throws InterruptedException {
        // first put the arm up.
        armRelease.release();
        _liftclaw.calibrateLift();
        Thread.sleep(1500);
        _liftclaw.runToPos(LiftClaw.LOW_POS);


        // move to the cone
        strafeDirection(22); // should put us clearly on the cone
        //get the place to end from the findMaxColor()
        int place_to_end = getColorSensorDevice().findMaxColor();
        telemetry.log().add("Place to end: "+place_to_end);
        telemetry.update();
        // now to move around a bit...
        // place the first cone
        strafeDirection(23);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        // now get a new one.....
        strafeDirection(12);
        //turnRobot(0);
        driveForward(24.5);

        pickNextCone();
        // place it on the pole
        driveReverse(25);
        strafeAntiDirection(12);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        switch (place_to_end) {
            case 1:
                strafeDirection(14);
                driveForward(22);
                break;
            case 3:
                strafeDirection(14);
                driveReverse(24);
                break;
        }
        turnRobot(turnDirection());
        light.off();
        requestOpModeStop();
    }

    public void runRight() throws InterruptedException {
        // first put the arm up.
        armRelease.release();
        _liftclaw.calibrateLift();
        Thread.sleep(1500);
        _liftclaw.runToPos(LiftClaw.LOW_POS);


        // move to the cone
        strafeDirection(22); // should put us clearly on the cone
        //get the place to end from the findMaxColor()
        int place_to_end = getColorSensorDevice().findMaxColor();
        telemetry.log().add("Place to end: "+place_to_end);
        telemetry.update();
        // now to move around a bit...
        // place the first cone
        strafeDirection(23.5);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        // now get a new one.....
        strafeDirection(13.5);
        //turnRobot(0);
        driveForward(26.5);

        pickNextCone();
        // place it on the pole
        driveReverse(26.5);
        strafeAntiDirection(13.5);
        driveForward(3);
        placeCone(1200);
        driveReverse(3);

        switch (place_to_end) {
            case 1:
                strafeDirection(14);
                driveReverse(24);
                break;
            case 3:
                strafeDirection(14);
                driveForward(22);
                break;
        }
        turnRobot(turnDirection());
        light.off();
        requestOpModeStop();
    }

    public void driveForward(double distanceInches) {
        _move.driveForward(distanceInches);
    }

    public void driveLeft(double distanceInches) {
        _move.driveLeft(distanceInches);
    }

    public void driveReverse(double distanceInches) {
        _move.driveReverse(distanceInches);
    }

    public void driveRight(double distanceInches) {
        _move.driveRight(distanceInches);
    }


    public void pickNextCone() {
        _liftclaw.clawClose();
        _liftclaw.runToPos(current_stack_height); // pick up cone
        _liftclaw.runToPos(LiftClaw.LOW_POS);
        updateStackHeight();
    }

    public void placeCone(long pos) {
        _liftclaw.clawOpen();
    }

    public void turnRobot(double direction) {

    }

    public void updateStackHeight() {
        current_stack_height -= LiftClaw.STACK_INCREMENT;
    }

}