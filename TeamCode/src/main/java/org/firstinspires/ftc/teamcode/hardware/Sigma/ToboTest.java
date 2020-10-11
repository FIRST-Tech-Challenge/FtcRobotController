package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

public class ToboTest extends Logger<ToboTest> implements Robot2 {
    private Telemetry telemetry;
    public SwerveChassis chassis;
    public CameraStoneDetector cameraStoneDetector;
    public FoundationHook foundationHook;
    public StoneGrabber stoneGrabber;
    public IntakeV3 intake;

    public enum SkystoneLocation {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public double motor_count = 0;
    public double auto_chassis_power = .6;

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, ProgramType autoColor) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;

//        cameraSystem = new CameraSystem(null);
//        cameraSystem.init(configuration.getHardwareMap());

        this.core = new CoreSystem();
        info("RoboSigma configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));
       chassis = new SwerveChassis(this.core).configureLogging("Swerve", logLevel); // Log.DEBUG
       chassis.enableRangeSensorTelemetry();
       chassis.enableShowColors();
       chassis.enableImuTelemetry();
        chassis.configure(configuration, ((autoColor != ProgramType.TELE_OP)), true);
        info("RoboSigma configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
//        if (auto) {
//            cameraStoneDetector = new CameraStoneDetector().configureLogging("CameraStoneDetector", logLevel);
//            cameraStoneDetector.configure(configuration, true);
//        }
        info("RoboSigma configure() after init cameraStoneDetector (run time = %.2f sec)", (runtime.seconds() - ini_time));

        foundationHook = new FoundationHook(this.core).configureLogging("FoundationHook", logLevel);
        foundationHook.configure(configuration, false);

        //stoneGrabber = new StoneGrabber(this.core).configureLogging("StoneGrabber", logLevel);
        //stoneGrabber.configure(configuration, false);

         intake = new IntakeV3(this.core).configureLogging("intakeV3", logLevel);
         intake.configure(configuration, false);

    }

    @Override
    public void reset(boolean auto) {
        if (chassis!=null) {
            chassis.reset();
            if (auto) {
                chassis.setupTelemetry(telemetry);
            }
        }
        if (intake!=null)
            intake.intakeDropInit();
    }

    public void end() {

    }

    @MenuEntry(label = "TeleOp", group = "Competition")
    public void mainTeleOp(EventManager em, EventManager em2) {
        telemetry.addLine().addData("(RS)", "4WD").setRetained(true)
                .addData("(RS) + (LS)", "2WD / Steer").setRetained(true);
        telemetry.addLine().addData("< (LS) >", "Rotate").setRetained(true)
                .addData("[LB]/[LT]", "Slow / Fast").setRetained(true);
        telemetry.addLine().addData("motor_count=", new Func<String>() {
            @Override
            public String value() {
                return String.format("%2.0f", motor_count);
            }
        });
        if (chassis!=null) chassis.setupTelemetry(telemetry);

        em.updateTelemetry(telemetry, 100);
//        if (!hanging.latchIsBusy()) {
//            hanging.resetLatch();
//        }

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.BOTH)) < 0.1) {
                    // right stick with idle left stick operates robot in "crab" mode
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH));
                    power *= power; // square power to stick
                    double heading = toDegrees(currentX, currentY);
                    double cur_heading = chassis.getCurHeading();
                    // invert headings less than -90 / more than 90
                    if ((Math.abs(cur_heading - heading) < 10) || (Math.abs(currentX) + Math.abs(currentY) < 0.1)) { // keep original heading
                        heading = cur_heading;
                    }
                    // dead zone mapping: [-120, -75] to -90
                    // dead zone mapping: [75, 120] to 90
                    if (heading>-120 && heading<-75) heading = -90;
                    if (heading>75 && heading<120) heading = 90;
                    if ((Math.abs(cur_heading-heading)==180) && Math.abs(heading)<=90) {
                        heading = cur_heading;
                        power = -1 * power;
                    }
                    if (Math.abs(heading) > 90) { // reduce rotation angle
                        heading -= Math.signum(heading) * 180;
                        power = -1 * power;
                    }
                    debug("sticksOnly(): straight, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, true);
                } else if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY))>0.2 && Math.abs(currentY)<0.1) {
                    // Orbit mode: right-stickY is small and both right-stickX left-StickY have value
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY));
                    double curvature = Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY));
                    power *= power * source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY); // square power to stick
                    chassis.orbit(power*powerAdjustment(source),curvature, source.isPressed(Button.START));
                } else {
                    // right stick with left stick operates robot in "car" mode
                    double heading = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY) * 90;
                    double power = currentY * Math.abs(currentY);
                    debug("sticksOnly(): right / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH)) < 0.2 &&
                        Math.abs(currentX) > 0.1) {
                    // left stick with idle right stick rotates robot in place
                    chassis.rotate(currentX * Math.abs(currentX) * powerAdjustment(source)*rotateRatio);
                } else if (source.getTrigger(Events.Side.RIGHT) < 0.2 && Math.abs(currentX) > 0.1) {
                    // right stick with left stick operates robot in "car" mode
                    double heading = currentX * 90;
                    double power = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
                    debug("sticksOnly(): left / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
                else if (chassis!=null){
                    chassis.stop();
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);


        // em: [RB] + [Y] for mineral dump combo (move slider to dump, intake box up, open box gate)
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.START) && source.isPressed(Button.BACK)) { // testing chassis speed
                    motor_count = chassis.driveStraightSec(1.0, 10, false);
                    return;
                } else if (source.isPressed(Button.LEFT_BUMPER) && source.isPressed(Button.RIGHT_BUMPER)) { // testing chassis speed
                    motor_count = chassis.driveStraightSec(1.0, 2, false);
                    return;
                }
                else if (source.isPressed(Button.BACK)) { // default scale up
                    if (chassis!=null) chassis.setDefaultScale(1.0);
                    return;
                }

            }
        }, Button.Y);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                intake.leftIntakeDropIn();
            }
        }, Button.DPAD_UP);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                intake.leftIntakeStop();
            }
        }, Button.DPAD_UP);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                intake.leftIntakeDropOut();
            }
        }, Button.DPAD_DOWN);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                intake.leftIntakeStop();
            }
        }, Button.DPAD_DOWN);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (intake!=null)
                    intake.intakeIn(!source.isPressed(Button.BACK));
            }
        }, Button.LEFT_BUMPER);

        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (intake != null) intake.intakeStop();
            }
        }, Button.LEFT_BUMPER);

        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // 0.2 is a dead zone threshold for the trigger

                if (current > 0.2) {
                    if (intake != null) intake.intakeOut(!source.isPressed(Button.BACK));
                } else {
                    if (intake != null) intake.intakeStop();
                }
            }
        }, Events.Side.LEFT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake != null) intake.intakeInOutCombo();
                } else if (source.isPressed(Button.BACK)) { // default scale back to 0.5
                    if (chassis!=null) chassis.setDefaultScale(0.7);
                }
            }
        }, Button.A);

    }



    @MenuEntry(label = "Auto Straight", group = "Test Chassis")
    public void testStraightSkyStone(EventManager em) {

        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                 if (auto_chassis_power>1) auto_chassis_power=1;
                } else {
                    chassis.driveStraightAuto(auto_chassis_power, 65, 0, 10000);
                    chassis.driveStraightAuto(auto_chassis_power, -7, 0, 10000);
                    chassis.driveStraightAuto(auto_chassis_power, 220, -90, 15000);
                    chassis.driveStraightAuto(auto_chassis_power, 260, 90, 15000);
                    chassis.driveStraightAuto(auto_chassis_power, 20, 0, 10000);
                    chassis.driveStraightAuto(auto_chassis_power, -5, 0, 10000);
                    chassis.driveStraightAuto(auto_chassis_power, 243, -90, 15000);//245?
                }
            }
        }, new Button[]{Button.Y});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                }
            }
        }, new Button[]{Button.A});


    }

    @MenuEntry(label = "Test Stone Grabber", group = "Test")
    public void testStoneGrabber(EventManager em) {

        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        stoneGrabber.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.B)) {
                    stoneGrabber.armOutCombo();
                }else if (source.isPressed(Button.X)) {
                    stoneGrabber.armInCombo(false, false);
                }else if (source.isPressed(Button.A)) {
                    stoneGrabber.grabStoneCombo();
                }else if (source.isPressed(Button.Y)) {
                    stoneGrabber.deliverStoneCombo(false);
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    stoneGrabber.liftToSafe();
                }
            }
        }, Button.LEFT_BUMPER);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (!source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.liftUp(source.isPressed(Button.BACK),true);
            }
        }, new Button[]{Button.Y});
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                stoneGrabber.liftStop();
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (!source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.liftDown(source.isPressed(Button.BACK), source.isPressed(Button.RIGHT_BUMPER));
            }
        }, new Button[]{Button.A});
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                stoneGrabber.liftStop();
            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (!source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.grabberAuto();
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (!source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.liftToSafe();
            }
        }, new Button[]{Button.B});

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)) > 0.2) {
                    // left stick with idle right stick rotates robot in place
                    if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY) > 0)
                        stoneGrabber.armUpInc(source.isPressed(Button.BACK));
                    else
                        stoneGrabber.armDownInc();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);
    }
    @MenuEntry(label = "Drive Straight", group = "Test Chassis")
    public void testStraight(EventManager em) {
        telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
                .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                double power = Math.max(Math.abs(currentX), Math.abs(currentY));
                double heading = toDegrees(currentX, currentY);
                debug("testStraight(): x: %+.2f, y: %+.2f, pow: %+.3f, head: %+.1f",
                        currentX, currentY, power, heading);

                if (source.isPressed(Button.LEFT_BUMPER) || source.isPressed(Button.RIGHT_BUMPER)) {
                    // constrain to 45 degree diagonals
                    heading = Math.signum(heading) * (Math.abs(heading) < 90 ? 45 : 135);
                } else {
                    // constrain to 90 degrees
                    heading = Math.round(heading / 90) * 90;
                }

                // adjust heading / power for driving backwards
                if (heading > 90) {
                    chassis.driveStraight(-1.0 * power, heading - 180);
                } else if (heading < -90) {
                    chassis.driveStraight(-1.0 * power, heading + 180);
                } else {
                    chassis.driveStraight(power, heading);
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);
    }

    @MenuEntry(label = "Rotate in Place", group = "Test Chassis")
    public void testRotate(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                chassis.rotate(currentX);
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
    }


    /**
     * Returns angle (-180 to 180 degrees) between positive Y axis
     * and a line drawn from center of coordinates to (x, y).
     * Negative values are to the left of Y axis, positive are to the right
     */
    private double toDegrees(double x, double y) {
        if (x == 0) return y >= 0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }

    /**
     * Returns power adjustment based on left bumper / left trigger state
     * Normal mode = 60% power
     * Left bumper down = slow mode (30% power)
     * Left trigger down = turbo mode (up to 100%) power
     */
    private double powerAdjustment(EventManager source) {
        double adjustment = chassis.getDefaultScale();  // default adjustment
        double trig_num = 0.0;
        if (source.isPressed(Button.RIGHT_BUMPER)) {
            // slow mode uses 30% of power
            adjustment = 0.25;
        } else if ((trig_num = source.getTrigger(Events.Side.RIGHT)) > 0.2) {
            // 0.2 is the dead zone threshold
            // turbo mode uses (100% + 1/2 trigger value) of power
            adjustment = 0.9 + 0.1 * trig_num * trig_num;
        }
        return adjustment;
    }

    public void getFirstSkyStone(SkystoneLocation skyStonePosition) throws InterruptedException{
        chassis.driveStraightAuto(auto_chassis_power, 65, 0, 10000);
        if(skyStonePosition == ToboTest.SkystoneLocation.LEFT){
            chassis.driveStraightAuto(auto_chassis_power, 1, 90, 10000);  // test to get exact numbers
        } else if(skyStonePosition == ToboTest.SkystoneLocation.CENTER){
            chassis.driveStraightAuto(auto_chassis_power, 1, 90, 10000);  // test to get exact numbers
        } else if(skyStonePosition == ToboTest.SkystoneLocation.RIGHT) {
            chassis.driveStraightAuto(auto_chassis_power, 1, 90, 10000);  // test to get exact numbers
        }
        //grab skystone
        int ss_pos = 1;
        if (skyStonePosition== ToboTest.SkystoneLocation.RIGHT)
            ss_pos = 3;
        else if (skyStonePosition== ToboTest.SkystoneLocation.CENTER)
            ss_pos = 2;
        chassis.driveStraightAuto(auto_chassis_power, 220 + 20 * ss_pos, -90, 15000);//probably too much
        //place skystone
    }
    public void getAnotherSkyStone(SkystoneLocation skyStonePosition, int stoneNum) throws InterruptedException{//stoneNum - how many stones ara we going to have after this trip
        int toTake;
        if (skyStonePosition == SkystoneLocation.LEFT || skyStonePosition == SkystoneLocation.UNKNOWN) {
            int [] a = {4, 2, 3, 5, 6};
            toTake = a[stoneNum - 2];
        } else if (skyStonePosition == SkystoneLocation.CENTER) {
            int [] a = {5, 1, 3, 4, 6};
            toTake = a[stoneNum - 2];
        } else {
            int[] a = {6, 1, 2, 4, 5};
            toTake = a[stoneNum - 2];
        }
        chassis.driveStraightAuto(auto_chassis_power, 260 + 20 * stoneNum, 90, 15000);//numbers - probably not correct
        chassis.driveStraightAuto(auto_chassis_power, 20, 0, 10000);
        chassis.driveStraightAuto(auto_chassis_power, -5, 0, 10000);
        //grab stone
        chassis.driveStraightAuto(auto_chassis_power, 243 + 20 * stoneNum, -90, 15000);
        // place stone on foundation

    }


}
