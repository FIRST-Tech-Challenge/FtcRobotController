package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.util.List;

import static java.lang.Thread.sleep;

public class ToboSigma extends Logger<ToboSigma> implements Robot2 {
    private Telemetry telemetry;
    public SwerveChassis chassis;
    public CameraStoneDetector cameraStoneDetector;
    public FoundationHook foundationHook;
    public IntakeV3 intake;
    public StoneGrabberV2 stoneGrabber;

    public class AutoPara {
        boolean isBlue = true;
        boolean laneFront = false;
        boolean offensive = false;
        boolean parkOnly = false;
        boolean isDone = false;

        public boolean isDone() {
            return isDone;
        }

        public boolean isBlue() {
            return isBlue;
        }

        public boolean isLaneFront() {
            return laneFront;
        }

        public boolean isOffensive() {
            return offensive;
        }

        public boolean isParkOnly() {
            return parkOnly;
        }
    }

    public class TensorPara {
        int iter = 2;
        int correctloc = 2; //1 = left; 2 = center; 3 = right
        boolean isDone = false;

        public int getIter() {
            return iter * 50;
        }

        public int correctLoc() {
            return correctloc;
        }

        public boolean isDone() {
            return isDone;
        }
    }

    public class CDist {
        double angle;
        double distance;

        public double getAngle() {
            return angle;
        }

        public double getDistance() {
            return distance;
        }
    }

    public AutoPara autoPara = null;
    public TensorPara tensorPara = null;

    public enum SkystoneLocation {
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    public enum CameraSource {
        INTERNAL, WEBCAM_RIGHT, WEBCAM_LEFT
    }

    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime runtimeAuto = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public double motor_count = 0;
    public double auto_chassis_power = .7;
    public double auto_chassis_dist = 150;
    public double auto_chassis_heading = -90;
    public double auto_chassis_power_slow = .4;
    public double auto_chassis_align_power = .22;
    public int stone_pos = 6;


    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    public double getAutoTimeLeft() {
        // initially assuming 29.5 second left
        return (29.5 - runtimeAuto.seconds());
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, ProgramType autoColor) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;

        autoPara = new AutoPara();
        tensorPara = new TensorPara();

        this.core = new CoreSystem();
        info("RoboSigma configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));
        chassis = new SwerveChassis(this.core).configureLogging("Swerve", logLevel); // Log.DEBUG
        if (autoColor == ProgramType.DIAGNOSIS) {
            chassis.enableImuTelemetry();
            chassis.enableRangeSensorTelemetry();
        }
        // Warning: MUST disable the following line during competition
        // chassis.enableRangeSensorTelemetry();//Comment out later

        chassis.configure(configuration, (autoColor != ProgramType.TELE_OP), true);
        info("RoboSigma configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
        if (autoColor != ProgramType.TELE_OP && autoColor != ProgramType.DIAGNOSIS) {
            cameraStoneDetector = new CameraStoneDetector().configureLogging("CameraStoneDetector", logLevel);
            // cameraStoneDetector.configure(configuration, CameraSource.INTERNAL);
            cameraStoneDetector.configure(configuration, CameraSource.WEBCAM_RIGHT);
            info("RoboSigma configure() after init cameraStoneDetector (run time = %.2f sec)", (runtime.seconds() - ini_time));
        }
        foundationHook = new FoundationHook(this.core).configureLogging("FoundationHook", logLevel);
        foundationHook.configure(configuration, (autoColor != ProgramType.TELE_OP));
        if (autoColor == ProgramType.DIAGNOSIS) {
            foundationHook.hookUp();
        }

        stoneGrabber = new StoneGrabberV2(this.core).configureLogging("StoneGrabber", logLevel);
        stoneGrabber.configure(configuration, (autoColor != ProgramType.TELE_OP));
        intake = new IntakeV3(this.core).configureLogging("IntakeV3", logLevel);
        intake.configure(configuration, (autoColor != ProgramType.TELE_OP));

    }

    @Override
    public void reset(boolean auto) {
        reset(auto, false);
    }

    public void end() {}

    public void reset(boolean auto, boolean armOut) {
        if (Thread.currentThread().isInterrupted()) return;
        if (chassis != null) {
            chassis.reset();
            if (auto) {
                // Display all sensors in auto only for debugging
                // chassis.setupTelemetry(telemetry);
            } else {
                chassis.changeChassisDrivingDirection();
            }
        }
        if (foundationHook != null) {
            foundationHook.reset(auto);
        }
        if (stoneGrabber != null)
            stoneGrabber.reset(auto, armOut);

        if (intake != null) {
            intake.reset(auto);
        }
    }

    @MenuEntry(label = "TeleOp", group = "Competition")
    public void mainTeleOp(EventManager em, EventManager em2) {
//        if (chassis.isTankDrive()) {
//            telemetry.addLine().addData("(RS) + (LS)", "Tank").setRetained(true);
//        } else {
//            telemetry.addLine().addData("(RS)", "4WD").setRetained(true)
//                    .addData("(RS) + (LS)", "2WD / Steer").setRetained(true);
//            telemetry.addLine().addData("< (LS) >", "Rotate").setRetained(true)
//                    .addData("[LB]/[LT]", "Slow / Fast").setRetained(true);
//        }
//        telemetry.addLine().addData("motor_count=", new Func<String>() {
//            @Override
//            public String value() {
//                return String.format("%2.0f", motor_count);
//            }
//        });
        if (chassis != null)
            chassis.setupTelemetry(telemetry);
        if (stoneGrabber != null)
            stoneGrabber.setupTelemetry(telemetry);
        if (intake != null)
            intake.setupTelemetry(telemetry);
        if (foundationHook != null)
            foundationHook.setupTelemetry(telemetry);

        em.updateTelemetry(telemetry, 100);

        // -----------------------------------
        // Teleop driver-1 control:
        // ------------------------------------
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (chassis.isTankDrive()) {
                    chassis.tankDrive(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY), currentY);
                } else if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.BOTH)) < 0.1) {
                    // right stick with idle left stick operates robot in "crab" mode
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY));
                    power = Math.max(power, Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY)));
                    power *= power; // square power to stick
                    double heading = toDegrees(currentX, currentY);
                    if (chassis.isReversed()) power *= -1;
                    double cur_heading = chassis.getCurHeading();
                    // invert headings less than -90 / more than 90
                    if ((Math.abs(cur_heading - heading) < 10) || (Math.abs(currentX) + Math.abs(currentY) < 0.1)) { // keep original heading
                        heading = cur_heading;
                    }
                    // dead zone mapping: [-120, -75] to -90
                    // dead zone mapping: [75, 120] to 90
                    if (heading > -120 && heading < -75) heading = -90;
                    else if (heading > 75 && heading < 120) heading = 90;
                    else if (cur_heading == 90 && heading > 90 && heading < 145) heading = 90;
                    else if (cur_heading == -90 && heading < -90 && heading > -145) heading = -90;
                    if ((Math.abs(cur_heading - heading) == 180) && Math.abs(heading) <= 90) {
                        heading = cur_heading;
                        power = -1 * power;
                    }
                    if (Math.abs(heading) > 90) { // reduce rotation angle
                        heading -= Math.signum(heading) * 180;
                        power = -1 * power;
                    }
                    debug("sticksOnly(): straight, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, true);
                } else {
                    // right stick with left stick operates robot in "car" mode
                    double heading = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY) * 90;
                    double power = currentY * Math.abs(currentY);
                    if (chassis.isReversed()) power *= -1;
                    debug("sticksOnly(): right / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (chassis.isTankDrive()) {
                    ; // do nothing
                } else if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH)) < 0.2) {
                    if (Math.abs(currentX) > 0.1) {
                        // left stick with idle right stick rotates robot in place
                        chassis.rotate(currentX * Math.abs(currentX) * powerAdjustment(source) * rotateRatio);
                    } else {
                        chassis.stop();
                    }
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (chassis.isTankDrive()) {
                    chassis.tankDrive(currentY, source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY));
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                double power = (source.isPressed(Button.RIGHT_BUMPER) ? -auto_chassis_power_slow : -auto_chassis_power);
                double curvature = Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY));
                chassis.orbit(power, curvature, source.isPressed(Button.START));
            }
        }, Button.DPAD_LEFT);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                chassis.stop();
            }
        }, Button.DPAD_LEFT);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                double power = (source.isPressed(Button.RIGHT_BUMPER) ? auto_chassis_power_slow : auto_chassis_power);
                double curvature = Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY));
                chassis.orbit(power, curvature, source.isPressed(Button.START));
            }
        }, Button.DPAD_RIGHT);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                chassis.stop();
            }
        }, Button.DPAD_RIGHT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                stoneGrabber.parkingServoAuto();
            }
        }, Button.DPAD_DOWN);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.A) || source.isPressed(Button.B) || source.isPressed(Button.Y) || source.isPressed(Button.X)) {
                    if (intake != null) intake.intakeStop();
                    return;
                }
                if (intake != null) {
                    if (source.isPressed(Button.DPAD_UP)) {
                        if (intake != null) intake.intakeDropAuto();
                    } else {
                        // intake.ingateOpen();
                        // chassis.setDefaultScale(chassis.DEFAULT_SLOW_SCALE);
                        chassis.setDefaultScale(0.25);
                        // intake.intakeIn(!source.isPressed(Button.BACK));
                        stoneGrabber.outGateClose();
                        intake.inTakeInCombo();
                    }
                }
            }
        }, Button.LEFT_BUMPER);

        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.A) || source.isPressed(Button.B) || source.isPressed(Button.Y) ||
                        source.isPressed(Button.X) || source.isPressed(Button.DPAD_UP)) {
                    return;
                }
                if (intake != null) {
                    final String taskName = "Intake In Combo";
                    while (!TaskManager.isComplete(taskName) && Thread.interrupted()) {
                        TaskManager.processTasks();
                    }
                    intake.intakeStop();
                    chassis.setDefaultScale(chassis.DEFAULT_FAST_SCALE);
                    if (!intake.feederModeCheck())
                        intake.ingateClose();
                }
            }
        }, Button.LEFT_BUMPER);

        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // 0.2 is a dead zone threshold for the trigger

                if (current > 0.2) {
                    if (intake != null) {
                        intake.intakeOut(!source.isPressed(Button.BACK));
                        intake.ingateOpen();
                    }
                } else {
                    if (intake != null) intake.intakeStop();
                }
            }
        }, Events.Side.LEFT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) { // default scale up
                    chassis.setDefaultScale(1.0);
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    stoneGrabber.armInReadyGrabCombo();
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    chassis.incDefaultScale();
                }
            }
        }, Button.Y);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake != null) intake.intakeDropAuto();
                } else if (source.isPressed(Button.BACK)) { // default scale toggle slow and fast
                    chassis.toggleSlowMode();
                } else if (source.isPressed(Button.LEFT_BUMPER)) { // same for driver-2 grab stone combos
                    if (stoneGrabber.isArmInside())
                        stoneGrabber.grabStoneInsideCombo();
                    else
                        stoneGrabber.grabStoneCombo();
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    chassis.decDefaultScale();
                } else if (!source.isPressed(button.START)) {
                    foundationHook.hookAuto();
                }
            }
        }, Button.A);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed((Button.DPAD_UP))) {
                    stoneGrabber.outGateOpen();
                    intake.ingatePush();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    stoneGrabber.armInCombo(!source.isPressed(Button.BACK), false);
                } else if (source.isPressed((Button.RIGHT_BUMPER))) {
                    intake.ingateAuto();
                } else if (source.isPressed(Button.BACK)) { // back-X swap driving direction
                    chassis.changeChassisDrivingDirection();
                } else {
                    stoneGrabber.grabberAuto();
                }
            }
        }, new Button[]{Button.X});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armOutCombo(0, false);
                else if (source.isPressed(Button.BACK)) {
                    stoneGrabber.grabInsideAndArmOutCombo(0, true);
                    // if (intake != null)
                    //    intake.feederModeAuto();
                } else if (source.isPressed(Button.RIGHT_BUMPER))
                    chassis.toggleTankDrive();
                else if (!source.isPressed(Button.START)) {
                    stoneGrabber.outGateOpen();
                    intake.ingatePush();
                }
            }
        }, new Button[]{Button.B});


        // ---------------------------------
        // Teleop Driver-2 control:
        // ---------------------------------

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake != null) intake.intakeDropAuto();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (stoneGrabber.isArmInside())
                        stoneGrabber.grabStoneInsideCombo();
                    else
                        stoneGrabber.grabStoneCombo();
                } else if (source.isPressed(Button.BACK)) // reset lifter encoder to 0
                    stoneGrabber.liftResetEncoder();
            }
        }, new Button[]{Button.A});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER)) {
                    intake.ingateOpen();
                    stoneGrabber.armInReadyGrabCombo();
                } else if (source.isPressed(Button.X))
                    stoneGrabber.releaseStoneCombo();
            }
        }, new Button[]{Button.Y});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInCombo(!source.isPressed(Button.BACK), false);
                else {
                    stoneGrabber.grabberAuto();
                    stoneGrabber.record_pos();
                }
            }
        }, new Button[]{Button.X});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.armOutReadyGrabAutoCombo();
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armOutCombo(0, false);
                else if (!source.isPressed(Button.START))
                    stoneGrabber.outGateAuto();
            }
        }, new Button[]{Button.B});

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)) > 0.2) {
                    // left stick with idle right stick rotates robot in place
                    if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY) > 0)
                        stoneGrabber.armUpInc();
                    else
                        stoneGrabber.armDownInc(source.isPressed(Button.BACK));
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY)) > 0.5) {
                    // left stick with idle right stick rotates robot in place
                    if (source.isPressed(Button.LEFT_BUMPER)) {
                        if (source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY) < 0)
                            stoneGrabber.outGateUpInc();
                        else stoneGrabber.outGateDownInc();
                    } else {
                        stoneGrabber.liftHold();
                    }
                } else if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY)) > 0.2) {
                    // left stick with idle right stick rotates robot in place
                    double ratio = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
                    if (ratio > 0) {
                        if (source.isPressed(Button.LEFT_BUMPER)) {
                            stoneGrabber.liftUpCombo();
                        } else {
                            stoneGrabber.liftUp(source.isPressed(Button.BACK), source.isPressed(Button.RIGHT_BUMPER), ratio);
                        }
                    } else {
                        stoneGrabber.liftDown(source.isPressed(Button.BACK), source.isPressed(Button.RIGHT_BUMPER), ratio);
                    }
                } else {
                    stoneGrabber.liftStop();
                    stoneGrabber.liftHold();
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY)) > 0.2) {
                    // left stick with idle right stick rotates robot in place
                    if (source.isPressed(Button.LEFT_BUMPER)) {
                        if (source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY) < 0)
                            stoneGrabber.outGateUpInc();
                        else stoneGrabber.outGateDownInc();
                    } else {
                        stoneGrabber.liftHold();
                    }
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.RIGHT);

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.capstoneRightInc();
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.regrabStoneCombo(true);
                else
                    stoneGrabber.parkingServoAuto();
            }
        }, new Button[]{Button.DPAD_RIGHT});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                stoneGrabber.parkingServoAuto();
            }
        }, new Button[]{Button.DPAD_DOWN});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.capstoneLeftInc();
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.regrabStoneCombo(false);
            }
        }, new Button[]{Button.DPAD_LEFT});

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    //stoneGrabber.grabCapStoneCombo();
                    stoneGrabber.grabberReleaseCapstone();
                }
            }
        }, new Button[]{Button.DPAD_UP});
    }

    public void setupTelemetryDiagnostics(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Test ", new Func<String>() {
            @Override
            public String value() {
                return String.format("Power=%.2f, dist=%.1f, heading=%.1f, rotate_degree=%.1f,pos=%1d\n",
                        auto_chassis_power, auto_chassis_dist, auto_chassis_heading, auto_rotate_degree, stone_pos);
            }
        });
    }

    public void intakeInAuto(boolean fast) {
        stoneGrabber.outGateClose();
        intake.intakeIn(fast);
    }

    public void setupTelemetry(Telemetry telemetry) {
        if (Thread.currentThread().isInterrupted()) return;
        Telemetry.Line line = telemetry.addLine();
        line.addData(" | <B> Team", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n", (autoPara.isBlue() ? "Blue" : "Red"));
            }
        });
        line.addData("<A> Lane", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n", (autoPara.isLaneFront() ? "Front" : "Back"));
            }
        });
        /*
        line.addData("Offensive", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isOffensive() ? "Yes" : "No"));
            }
        });
        */
        line.addData("<Y> Mode", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n", (autoPara.isParkOnly() ? "Park Only" : "Foundation and PArk"));
            }
        });
    }

    @MenuEntry(label = "Auto Backup", group = "Competition-Auto")
    public void AutoBackup(EventManager em) {
        telemetry.addLine().addData(" | <X>", "Done").setRetained(true);
        setupTelemetry(telemetry);

        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.laneFront = !autoPara.laneFront;
            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.parkOnly = !autoPara.parkOnly;
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.isDone = true;
            }
        }, new Button[]{Button.X});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.isBlue = !autoPara.isBlue;
            }
        }, new Button[]{Button.B});

    }

    @MenuEntry(label = "Auto drive straight", group = "Test Chassis")
    public void testStraightSkyStone(EventManager em) {
        if (Thread.interrupted()) return;
        telemetry.addLine().addData("(BACK) Y/A", "+/- Power(%.2f)", auto_chassis_power).setRetained(true);
        telemetry.addLine().addData("(BACK)(L-BUMP) X/B", "+/- dist(%.2f)", auto_chassis_dist).setRetained(true);
        telemetry.addLine().addData("(BACK) D-UP/DOWN", "+/- heading(%.2f)", auto_chassis_heading).setRetained(true);
        setupTelemetryDiagnostics(telemetry);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;

                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    chassis.rotateTo(.5, 90);
                    int side = 1;
                    boolean isBlue = true;
                    int toTake = stone_pos;
                    intake.intakeDropDown();
                    double distBack = chassis.getDistance(SwerveChassis.Direction.BACK);
                    double distLeft = side * chassis.getDistance(side == 1 ? SwerveChassis.Direction.LEFT_HI : SwerveChassis.Direction.RIGHT_HI);
                    align(distBack, distLeft, isBlue ? 27 + (6 - toTake) * 20.3 : 29 + (6 - toTake) * 20.3, isBlue ? 73 : -71);
                    sleep(5000);
                    chassis.rotateTo(.5, isBlue ? +45 : -45);
                    if (Thread.currentThread().isInterrupted()) return;
                    intakeInAuto(true);
                    chassis.driveAuto(.4, isBlue ? -35 : -33, 0, 2000);//suck in the stone
                    //******wait till stone is in
                    long iniTime = System.currentTimeMillis();
                    while (!chassis.stoneCollected() && (System.currentTimeMillis() - iniTime) < 800) {
                        if (Thread.interrupted()) return;
                    }
                    if (Thread.currentThread().isInterrupted()) return;
                    intake.ingateClose();
                    if (Thread.currentThread().isInterrupted()) return;
                    intake.intakeStop();
                    //******
                    chassis.driveAuto(.9, isBlue ? 30 : 41, 0, 2000);//was 30
                    if (Thread.currentThread().isInterrupted()) return;
                    chassis.rotateTo(.5, (isBlue ? 90 : -88));//was 90.5
                    // hereeeeee
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    int side = 1;// blue
                    double dist = chassis.getDistance(side == 1 ? SwerveChassis.Direction.LEFT_HI : SwerveChassis.Direction.RIGHT_HI);
                    CDist move = diagonalMove(260, -dist + 70);
                    chassis.driveAuto(.5, move.distance, Math.max(Math.min(move.angle, 3), -3), 4000);
                } else {
                    long iniTime = System.currentTimeMillis();
                    chassis.driveAuto(auto_chassis_power, auto_chassis_dist, auto_chassis_heading, 10000);
                    telemetry.addLine(System.currentTimeMillis() - iniTime + "");
                }
            }
        }, new Button[]{Button.Y});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (stone_pos == 6) stone_pos = 4;
                    else stone_pos++;
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
//                    deliverAndPark2SS(true);
                }
            }
        }, new Button[]{Button.A});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_dist += 1;
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    auto_chassis_dist += 10;
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_dist -= 1;
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    auto_chassis_dist -= 10;
                }
            }
        }, new Button[]{Button.B});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_heading -= 10;
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    auto_chassis_heading -= 5;
                }
                if (auto_chassis_heading < -90) auto_chassis_heading = -90;
            }
        }, new Button[]{Button.DPAD_DOWN});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_heading += 10;
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    auto_chassis_heading += 5;
                }
                if (auto_chassis_heading > 90) auto_chassis_heading = 90;
            }
        }, new Button[]{Button.DPAD_UP});
    }

    public double auto_rotate_degree = 90;

    @MenuEntry(label = "Auto Rotation", group = "Test Chassis")
    public void testRotationSkyStone(EventManager em) {
        if (Thread.interrupted()) return;
        telemetry.addLine().addData("(BACK) Y/A", "+/- Power(%.2f)", auto_chassis_power).setRetained(true);
        telemetry.addLine().addData("(BACK) X/B", "+/- degree(%.2f)", auto_rotate_degree).setRetained(true);
        chassis.setupTelemetry(telemetry);
        setupTelemetryDiagnostics(telemetry);
        chassis.enableImuTelemetry();
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.rotateTo(auto_chassis_power, auto_rotate_degree, 5000);
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
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 10;
                    if (auto_rotate_degree > 150) auto_rotate_degree = 150;
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 10;
                    if (auto_rotate_degree < -150) auto_rotate_degree = -150;
                }
            }
        }, new Button[]{Button.B});

    }

    // @MenuEntry(label = "New Auto Straight", group = "Test Chassis")
    public void testStraightNewSkyStone(EventManager em) {
        if (Thread.interrupted()) return;

        try {
            chassis.tl = telemetry;
            //chassis.driveStraightAutoRunToPosition(.4, 200, 0, 10000, telemetry);
            chassis.tl = telemetry;
            chassis.driveStraightAutoRunToPosition(.6, 150, -90, 10000);
            sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, 150, 90, 10000);
            sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, 150, 0, 10000);
            sleep(500);
            chassis.driveStraightAutoRunToPosition(.6, -150, 0, 10000);
        } catch (InterruptedException e) {

        }


        /*
        telemetry.addLine().addData(" < (BACK) >", "Power(%.2f)", auto_chassis_power).setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.driveStraightAutoNew(auto_chassis_power, 100, 0, 10000);

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
        *
         */
    }

    //@MenuEntry(label = "Drive Straight", group = "Test Chassis")
    public void testStraight(EventManager em) {
        if (Thread.interrupted()) return;
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

    // @MenuEntry(label = "Rotate in Place", group = "Test Chassis")
    public void testRotate(EventManager em) {
        if (Thread.interrupted()) return;
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

    @MenuEntry(label = "Crab", group = "Test Chassis")
    public void testCrab(EventManager em) {
        if (Thread.interrupted()) return;
        telemetry.addLine().addData(" < (LS) >", "Direction").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                chassis.driveStraightAuto(.5, 120, 90 * Math.signum(currentX), 100000);
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
    }

    @MenuEntry(label = "Wheel Intake", group = "Test Chassis")
    public void testWheelIntake(EventManager em) {
        if (Thread.interrupted()) return;
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                chassis.rotateTo(.6, 90);
                stoneGrabber.armInReadyGrabCombo();
                intake.intakeDropDown();
                intake.ingateOpen();
                chassis.rotateTo(.3, 90);
                double dist = chassis.getDistance(SwerveChassis.Direction.RIGHT);
                chassis.driveAuto(.3, -dist + 63, -90, 2000);
                chassis.driveAuto(.6, -260, 0, 5000);
                chassis.rotateTo(.3, 90);
                dist = chassis.getDistance(SwerveChassis.Direction.LEFT);
                chassis.driveAuto(.3, dist - 15, -90, 2000);
                sleep(200);
                dist = chassis.getDistance(SwerveChassis.Direction.BACK);
                chassis.driveAuto(.3, -dist + 30, 0, 2000);

                chassis.rotateTo(.6, 135);
                intake.intakeIn(true);
                chassis.driveAuto(auto_chassis_power_slow, -45, 0, 2000);
                chassis.driveAuto(auto_chassis_power, 37, 0, 2000);
                chassis.rotateTo(.3, 90);
                intake.intakeStop();
                intake.ingateClose();
                stoneGrabber.grabStoneInsideCombo();
                chassis.rotateTo(.6, 90);
                stoneGrabber.armOutCombo(2, true);
                chassis.driveAuto(.6, 250, 0, 5000);
                stoneGrabber.grabberOpenAuto();
                chassis.driveAuto(.5, -145, 5, 5000);
                stoneGrabber.lifterDownCombo();

            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.Y) && source.isPressed(Button.BACK)) {
                    // intake drop In/out
                    if (intake != null) intake.intakeDropAuto();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (stoneGrabber.isArmInside())
                        stoneGrabber.grabStoneInsideCombo();
                    else
                        stoneGrabber.grabStoneCombo();
                } else if (source.isPressed(Button.BACK)) // reset lifter encoder to 0
                    stoneGrabber.liftResetEncoder();
            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInReadyGrabCombo();
                else if (source.isPressed(Button.X))
                    stoneGrabber.releaseStoneCombo();
                else if (source.isPressed(Button.BACK)) {
                    if (stoneGrabber != null) stoneGrabber.deliverStoneThrowCombo();
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armInCombo(!source.isPressed(Button.BACK), false);
                else
                    stoneGrabber.grabberAuto();
            }
        }, new Button[]{Button.X});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER))
                    stoneGrabber.armOutCombo(1.0, true);
                else if (source.isPressed(Button.LEFT_BUMPER))
                    stoneGrabber.armOutCombo();
                else if (!source.isPressed(Button.START))
                    stoneGrabber.outGateAuto();
            }
        }, new Button[]{Button.B});
    }

    public void wheelIntakeFirstStone(SkystoneLocation ssLoc, boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        stoneGrabber.armInReadyGrabCombo();
        if (Thread.currentThread().isInterrupted()) return;
        intake.intakeDropDown();
        if (Thread.currentThread().isInterrupted()) return;
        intake.ingateOpen();
        if (Thread.currentThread().isInterrupted()) return;
        if (ssLoc == SkystoneLocation.LEFT) {
            chassis.driveAuto(.6, isBlue ? -74 : -74, -12, 5000);//going to a stone
        } else if (ssLoc == SkystoneLocation.RIGHT) {
            chassis.driveAuto(.6, isBlue ? -70.5 : -70, -4, 5000);//going to a stone
        } else {
            chassis.driveAuto(.6, isBlue ? -70.5 : -71, 4, 5000);//going to a stone
        }
        if (Thread.currentThread().isInterrupted()) return;
        //chassis.driveAuto(.6, 45, 90, 5000);
        intakeInAuto(true);
        //rotate skew to get ready wheel intake
        if (Thread.currentThread().isInterrupted()) return;
        if (ssLoc == SkystoneLocation.LEFT) {
            chassis.rotateTo(.4, -21, 2000, true, false);
        } else if (ssLoc == SkystoneLocation.CENTER) {
            chassis.rotateTo(.4, -21, 2000, true, false);
        } else {
            chassis.rotateTo(.4, 21, 2000, true, false);
        }
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.4, -12, 0, 5000);//go take the stone
        if (Thread.currentThread().isInterrupted()) return;

        long iniTime = System.currentTimeMillis();
        while (!chassis.stoneCollected() && (System.currentTimeMillis() - iniTime) < 800) {
            if (Thread.interrupted()) return;
        }
        intake.ingateClose();
        if (Thread.currentThread().isInterrupted()) return;
        intake.intakeStop();
        //chassis.rotateTo(.5, 0);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.8, 21, 0, 5000);//was 21
        if (Thread.currentThread().isInterrupted()) return;
        chassis.rotateTo(.7, isBlue ? 90 : -90, 3000);//facing the building Zone
        if (Thread.currentThread().isInterrupted()) return;
        stoneGrabber.grabStoneInsideCombo();
        if (Thread.currentThread().isInterrupted()) return;
        if (ssLoc == SkystoneLocation.LEFT) {
            chassis.driveAuto(.95, isBlue ? 220 : 220, 0, 3000);//going to the foundation
        } else if (ssLoc == SkystoneLocation.CENTER) {
            chassis.driveAuto(.95, isBlue ? 235 : 200, 0, 3000);//going to the foundation
        } else {
            chassis.driveAuto(.95, isBlue ? 230 : 210, 0, 3000);//going to the foundation
        }
        if (Thread.currentThread().isInterrupted()) return;
        chassis.rotateTo(.7, 179.9, 3000, true, false);
    }

    /**
     * public void wheelIntakeFirstStoneRed(SkystoneLocation ssLoc) throws InterruptedException {
     * if (Thread.currentThread().isInterrupted()) return;
     * stoneGrabber.armInReadyGrabCombo();
     * intake.intakeDropDown();
     * intake.ingateOpen();
     * if (Thread.currentThread().isInterrupted()) return;
     * if (ssLoc == SkystoneLocation.LEFT) {
     * chassis.driveAuto(.6, -75, -12, 5000);//going to a stone
     * } else {
     * chassis.driveAuto(.6, -72.5, 0, 5000);//going to a stone
     * }
     * if (Thread.currentThread().isInterrupted()) return;
     * //chassis.driveAuto(.6, 45, 90, 5000);
     * intakeInAuto(true);
     * //rotate skew to get ready wheel intake
     * if (ssLoc == SkystoneLocation.LEFT) {
     * chassis.rotateTo(.35, -23);
     * } else if (ssLoc == SkystoneLocation.CENTER) {
     * chassis.rotateTo(.35, -15);
     * } else {
     * chassis.rotateTo(.35, 20);
     * }
     * if (Thread.currentThread().isInterrupted()) return;
     * chassis.driveAuto(.4, -10, 0, 5000);//go take the stone
     * long iniTime = System.currentTimeMillis();
     * while (!chassis.stoneCollected() && (System.currentTimeMillis() - iniTime) < 500) {
     * if (Thread.interrupted()) return;
     * }
     * if (Thread.currentThread().isInterrupted()) return;
     * intake.ingateClose();
     * intake.intakeStop();
     * //chassis.rotateTo(.5, 0);
     * chassis.driveAuto(.6, 25, 0, 5000);
     * if (Thread.currentThread().isInterrupted()) return;
     * intake.intakeStop();
     * intake.ingateClose();
     * chassis.rotateTo(.7, -90, 3000);//facing the building Zone
     * if (Thread.currentThread().isInterrupted()) return;
     * stoneGrabber.grabStoneInsideCombo();
     * chassis.driveAuto(.6, 230, 0, 3000);//going to the foundation
     * if (Thread.currentThread().isInterrupted()) return;
     * chassis.rotateTo(.7, -179.9, 3000);
     * }
     */

    @MenuEntry(label = "Wheel Auto Blue", group = "Test Chassis")
    public void testWheelAutoBlue(EventManager em) {
        if (Thread.interrupted()) return;
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                wheelIntakeFirstStone(SkystoneLocation.CENTER, true);
            }
        }, new Button[]{Button.Y});//center
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                wheelIntakeFirstStone(SkystoneLocation.LEFT, true);
            }
        }, new Button[]{Button.X});//left
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                wheelIntakeFirstStone(SkystoneLocation.RIGHT, true);
            }
        }, new Button[]{Button.B});//right
    }

    @MenuEntry(label = "Wheel Auto Red", group = "Test Chassis")
    public void testWheelAutoRed(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                wheelIntakeFirstStone(SkystoneLocation.CENTER, false);
            }
        }, new Button[]{Button.Y});//center
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                wheelIntakeFirstStone(SkystoneLocation.LEFT, false);
            }
        }, new Button[]{Button.X});//left
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                wheelIntakeFirstStone(SkystoneLocation.RIGHT, false);
            }
        }, new Button[]{Button.B});//right
    }

    @MenuEntry(label = "New Auto", group = "Test Chassis")
    public void testNewAuto(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                stoneGrabber.armInReadyGrabCombo();
                intake.intakeDropDown();
                intake.ingateOpen();
                chassis.driveAuto(.6, -73, 0, 5000);//////////////////////////////////////going to a stone
                //chassis.driveAuto(.6, 45, 90, 5000);
                intake.intakeIn(true);
                chassis.rotateTo(.35, -20);//////////////////////////////////////////////////////rotating to a stone
                chassis.driveAuto(.4, -10, 0, 5000);
                //sleep(1000);
                //chassis.rotateTo(.5, 0);
                chassis.driveAuto(.6, 15, 0, 5000);
                intake.intakeStop();
                intake.ingateClose();
                chassis.rotateTo(.7, 90, 3000);
                stoneGrabber.grabStoneInsideCombo();
                chassis.driveAuto(.6, 220, 0, 3000);////////////////// going to the foundation
                chassis.rotateTo(.7, 179.5, 3000);
                rotateFoundation(true);
                wheelIntakeSecondStone(2, 2, true);

            }
        }, new Button[]{Button.Y});//center
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                stoneGrabber.armInReadyGrabCombo();
                intake.intakeDropDown();
                intake.ingateOpen();
                chassis.driveAuto(.6, -73, 0, 5000);//////////////////////////////////////going to a stone
                //chassis.driveAuto(.6, 45, 90, 5000);
                intake.intakeIn(true);
                chassis.rotateTo(.35, 20);//////////////////////////////////////////////////////rotating to a stone
                chassis.driveAuto(.4, -10, 0, 5000);
                //sleep(1000);
                //chassis.rotateTo(.5, 0);
                chassis.driveAuto(.6, 15, 0, 5000);
                intake.intakeStop();
                intake.ingateClose();
                chassis.rotateTo(.7, 90, 3000);
                stoneGrabber.grabStoneInsideCombo();
                chassis.driveAuto(.6, 220, 0, 3000);////////////////// going to the foundation
                chassis.rotateTo(.7, 179.5, 3000);
                rotateFoundation(true);
                wheelIntakeSecondStone(2, 3, true);


            }
        }, new Button[]{Button.X});//right
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                stoneGrabber.armInReadyGrabCombo();
                intake.intakeDropDown();
                intake.ingateOpen();
                chassis.driveAuto(.6, -78, -12, 5000);//////////////////////////////////////going to a stone
                //chassis.driveAuto(.6, 45, 90, 5000);
                intake.intakeIn(true);
                chassis.rotateTo(.35, -23);//////////////////////////////////////////////////////rotating to a stone
                chassis.driveAuto(.4, -10, 0, 5000);
                //sleep(1000);
                //chassis.rotateTo(.5, 0);
                chassis.driveAuto(.6, 15, 0, 5000);
                intake.intakeStop();
                intake.ingateClose();
                chassis.rotateTo(.7, 90, 3000);
                stoneGrabber.grabStoneInsideCombo();
                chassis.driveAuto(.6, 200, 0, 3000);////////////////// going to the foundation
                chassis.rotateTo(.7, 179.5, 3000);
                rotateFoundation(true);
                wheelIntakeSecondStone(2, 1, true);

            }
        }, new Button[]{Button.B});//left


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

    public void waitAndProcessTask(long timeMillis) {
        long iniTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - iniTime < timeMillis) {
            TaskManager.processTasks();
        }
    }

    //assume: arm is out, grabber is open
    public double grabStoneAndDeliverOnFoundation(double stoneX, boolean firstStone, boolean isBlue) throws InterruptedException {
        if (firstStone) {
//            stoneGrabber.grabStoneComboAuto();
            stoneGrabber.grabStoneComboHigher();
            while (!TaskManager.isComplete("Grab Stone Combo") && !Thread.interrupted()) {
                TaskManager.processTasks();
            }
            //====================parallelized region===================
//        stoneGrabber.armInCombo(true, true);
//            chassis.driveStraightAuto(.35, isBlue ? -7 : -7, 0, 1000);
        } else {

            double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT), chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT));
            if (dist - 14 > 1) {
                chassis.driveStraightAuto(.35, dist - 14, 0, 1000);////?
            } else if (dist - 14 < -1) {
                chassis.driveStraightAuto(.35, dist - 9, 0, 1000);////?
            }
            stoneGrabber.grabberOpenAuto();
            stoneGrabber.grabStoneComboHigher();
            while (!TaskManager.isComplete("Grab Stone Combo") && !Thread.interrupted()) {
                TaskManager.processTasks();
            }
            //====================parallelized region===================
//            chassis.driveStraightAutoRunToPosition(.35, firstStone ? -8 : -10, 0, 1000);

        }
        double distanceToBack = chassis.getDistance(SwerveChassis.Direction.BACK);
        chassis.driveStraightAuto(0.35, 65 - distanceToBack, 0, 1000);
        chassis.rotateTo(.6, isBlue ? -90 : +90);
        stoneGrabber.lifterDownCombo();
        waitAndProcessTask(200);
        chassis.driveStraightAutoRunToPositionNoIMU(0.75, 135 - stoneX, 0, 5000);//used to use IMU
        //==========================arriving before foundation=================================

        double disLeft = chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT);
        double disRight = chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT);
        double distanceToFoundation = Math.min(disLeft, disRight) > 70 ? 0 : Math.min(disLeft, disRight);
//        telemetry.addData("distance left",disLeft);
//        telemetry.addData("distance right",disRight);
//        telemetry.addData("dis to foundation",distanceToFoundation);
//        telemetry.update();
//        sleep(100);
        if (distanceToFoundation > 5) {
            chassis.driveStraightAuto(0.5, distanceToFoundation - 5, 0, 2000);
        }

        if (firstStone) {
            stoneGrabber.deliverStoneThrowComboAuto();//throw if first stone
            //stoneGrabber.armInComboAuto(false);//no para
            stoneGrabber.armInCombo(false, true);
            waitAndProcessTask(1000);
        } else {
            stoneGrabber.deliverStoneComboAuto();//don't throw if second stone
            stoneGrabber.armInCombo(false, true);
        }

        //let it finish


//        chassis.rotateTo(auto_chassis_align_power,-90);
        return 135 + distanceToFoundation;
    }


    public void approachStone(ToboSigma.SkystoneLocation skyStonePosition, boolean isBlue, boolean isLeft) throws InterruptedException {
        //=================================Parallelized region=======================================
//        stoneGrabber.armOutComboAuto();
        stoneGrabber.armOutCombo();
        chassis.driveStraightAutoRunToPosition(.6, 46, 0, 10000);
        while (!TaskManager.isComplete("Arm Out Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }
        //===========================================================================================
        stoneGrabber.grabberOpenAuto();
        // go to stones
        foundationHook.hookUp();

//        chassis.rotateTo(.25, 0);
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT)) - 12;
//        if (skyStonePosition != SkystoneLocation.UNKNOWN)
//            dist -= 2;
//        if (dist > 17) dist = 17;
//        chassis.driveStraightAutoRunToPosition(.6, dist, 0, 1000);
        if (dist - 14 > 1) {
            chassis.driveStraightAuto(.35, dist - 14, 0, 1000);////?
        } else if (dist - 14 < -1) {
            chassis.driveStraightAuto(.35, dist - 9, 0, 1000);////?
        }
    }

    //assumption: arm is out when the function is called
    public void oneMoreStone(boolean onQarry, double initialX, double stoneX, double dumpingX, int StoneId) throws InterruptedException {
        if (!onQarry) {
            //if in building zone, retract arm to pass sky bridge
            stoneGrabber.armInComboAuto(false);
        }

        if (!onQarry) {
            //if in building zone, passing the bridge and take out the arm midway
            //===================parallelized region===================
            stoneGrabber.armOutCombo();
            chassis.driveStraightAutoRunToPosition(0.75, Math.abs(initialX - stoneX), initialX > stoneX ? 90 : -90, StoneId == 5 ? 0.6 : 0.4, 5000);
            while (!TaskManager.isComplete("Arm Out Combo") && !Thread.interrupted()) {
                TaskManager.processTasks();
            }
            //=========================================================
            stoneGrabber.grabberOpenAuto();
            chassis.driveStraightAutoRunToPosition(.35, 13, 0, 1000);
        } else {
            chassis.driveStraightAutoRunToPosition(0.75, Math.abs(initialX - stoneX), initialX > stoneX ? 90 : -90, 5000);
        }

        stoneGrabber.grabStoneComboAuto();
        //====================parallelized region===================
        stoneGrabber.armInCombo(true, true);
        chassis.driveStraightAutoRunToPosition(.35, -13, 0, 1000);
//        chassis.rotateTo(auto_chassis_align_power, 0);
        long iniTime = System.currentTimeMillis();
        if (StoneId == 5 || StoneId == 4)
            while (System.currentTimeMillis() - iniTime < 150) {
                TaskManager.processTasks();
            }
        chassis.driveStraightAutoRunToPosition(0.75, dumpingX - stoneX, -90, 5000);
        //===========================================================

        stoneGrabber.armOutComboAuto();
        stoneGrabber.deliverStoneComboAuto();
    }


    public void repositioning(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        foundationHook.hookDown();

        Thread.sleep(400);
        double dist = Math.min(130, Math.max(chassis.getDistance(SwerveChassis.Direction.BACK), 115));

        //chassis.driveStraightAutoRunToPosition(auto_chassis_power/2, -dist - 10*(1- auto_chassis_power) , 7* side, 10000);
        //=================================Parallelized region=======================================
//        stoneGrabber.deliverStoneComboAuto();
        stoneGrabber.deliverStoneCombo(true);
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, -dist, 7 * side, 4000);//        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
        while (!TaskManager.isComplete("Deliver Stone Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }
        //===========================================================================================
        foundationHook.hookUp();

        chassis.rotateTo(auto_chassis_align_power, 0);

        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
        if (dist > 1) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, -Math.min(dist, 10), 0, 300);
        }

        dist = isBlue ? chassis.getDistance(SwerveChassis.Direction.LEFT) : chassis.getDistance(SwerveChassis.Direction.RIGHT);

        if (dist > 128) {
            dist = 20;
        }

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 90 - dist, 90 * side, 1500);
        // stoneGrabber.armInComboAuto(false);
    }


    public void grabAndPark(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;

        foundationHook.hookDown();
        sleep(400);
        double dist = Math.max(chassis.getDistance(SwerveChassis.Direction.BACK), 118);

        //chassis.driveStraightAutoRunToPosition(auto_chassis_power/2, -dist - 10*(1- auto_chassis_power) , 7* side, 10000);
        //=================================Parallelized region=======================================
//        stoneGrabber.deliverStoneComboAuto();
        stoneGrabber.deliverStoneCombo(true);
//        chassis.driveStraightAutoRunToPosition(0.7, -1.05*dist/Math.cos(Math.PI*7.0/180.0), 7 * side, 5000);
        chassis.driveStraightAutoRunToPositionNoIMU(.4, -dist, 5 * side, 5000);//        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
//        chassis.driveStraightAutoRunToPosition(0.6, -dist, 0, 5000);
        while (!TaskManager.isComplete("Deliver Stone Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }
        //===========================================================================================

        foundationHook.hookUp();

        chassis.rotateTo(.25, 0);

        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);

        if (dist > 128) {
            dist = 20;
        }

        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 100 - dist, 90 * side, 1500);
        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        if (dist < 95) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, Math.min(50, 95 - dist), 90 * side, 1500);
            dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        }
        stoneGrabber.armInComboAuto(false);

        if ((isBlue && chassis.getDistance(SwerveChassis.Direction.RIGHT) > 60) || (!isBlue && chassis.getDistance(SwerveChassis.Direction.LEFT) > 60)) {
            //chassis.driveStraightAutoRunToPosition(.4, 40, 90* side, 1500);
            chassis.driveStraightAutoRunToPosition(.6, 55, 90 * side, 1500);
        } else {
            chassis.driveStraightAutoRunToPosition(.6, 50, 0, 1500);
            chassis.rotateTo(auto_chassis_align_power, 0);
            chassis.driveStraightAutoRunToPosition(.6, 55, 90 * side, 1500);
        }

    }


    public void parkAfterRotate(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        chassis.driveStraightAutoRunToPosition(.4, 85, side * 90, 3000);

    }

    public void getOneStone(ProgramType color) throws InterruptedException {
        stoneGrabber.armOutCombo();
        while (!TaskManager.isComplete("Arm Out Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }
        stoneGrabber.grabberOpenAuto();
        chassis.driveAuto(0.5, 72, 0, 5000);

        stoneGrabber.grabStoneComboAutoHigher();
//        stoneGrabber.armInCombo(true, true);
        chassis.driveAuto(0.6, color == ProgramType.AUTO_BLUE ? -60 : -50, 0, 2000);
    }

    public int getFirstSkyStoneDefense(ToboSigma.SkystoneLocation skyStonePosition, boolean isBlue, boolean safe) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return 0;
        int side = isBlue ? 1 : -1;
        stoneGrabber.armOutCombo();
        if (Thread.currentThread().isInterrupted()) return 0;
        chassis.driveAuto(.6, 54, 0, 10000);
        if (Thread.currentThread().isInterrupted()) return 0;
        stoneGrabber.grabberOpenAuto();
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT)) - (isBlue ? 17 : 17);

        if (dist > 29) dist = 29;
        while (!TaskManager.isComplete("Arm Out Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }

        foundationHook.hookUp();

        if (Thread.currentThread().isInterrupted()) return 0;
        boolean ready_grab = false;
        if (skyStonePosition == SkystoneLocation.UNKNOWN) { // use color sensor to detect the skystone
            // go to stones
            stoneGrabber.armOutReadyGrabAutoCombo();
            chassis.driveAuto(.3, dist, 0, 1000);
            ready_grab = true;
            sleep(500);
            skyStonePosition = chassis.getSkystonePositionColor(!isBlue); // using color sensors need to be close enough to the stones
        }

//        telemetry.addData("skeystone=","%s",skyStonePosition.toString());
//        telemetry.update();
//        core.yield_for(5);
        if (Thread.currentThread().isInterrupted()) return 0;

        if (!isBlue) { // Red side
            if ((skyStonePosition == SkystoneLocation.LEFT) || skyStonePosition == SkystoneLocation.UNKNOWN) {
                chassis.driveAuto(auto_chassis_power_slow, 13, 90 * side, 500);  // test to get exact numbers
            } else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER) {
                chassis.driveAuto(auto_chassis_power_slow, 7, -90 * side, 700);  // test to get exact numbers
            } else { // if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT) {
                chassis.driveAuto(auto_chassis_power_slow, 27, -90 * side, 1000);  // test to get exact numbers
            }
        } else { // Blue side
            if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT || skyStonePosition == SkystoneLocation.UNKNOWN) {
                chassis.driveAuto(auto_chassis_power_slow, 8, 90 * side, 500);  // test to get exact numbers
            } else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER) {
                chassis.driveAuto(auto_chassis_power_slow, 13, -90 * side, 700); // test to get exact numbers
            } else { // skyStonePosition == ToboSigma.SkystoneLocation.LEFT
                chassis.driveAuto(auto_chassis_power_slow, 30, -90 * side, 1000);  // test to get exact numbers
            }
        }
        if (Thread.currentThread().isInterrupted()) return 0;

        if (!ready_grab) { // get close to the stone grab position
            stoneGrabber.armOutReadyGrabAutoCombo();
            chassis.driveAuto(.3, dist, 0, 1000);
        }

        //grab stone
        if (Thread.currentThread().isInterrupted()) return 0;

        stoneGrabber.grabStoneComboAuto();
        //chassis.driveStraightAutoRunToPosition(.3, -8, 0, 1000);

        int ss_pos = 1;
        if (isBlue) {
            if (skyStonePosition == ToboSigma.SkystoneLocation.RIGHT)
                ss_pos = 3;
            else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER)
                ss_pos = 2;
        } else { // red
            if (skyStonePosition == ToboSigma.SkystoneLocation.LEFT)
                ss_pos = 3;
            else if (skyStonePosition == ToboSigma.SkystoneLocation.CENTER)
                ss_pos = 2;
        }


        // go to foundation


        //=================================Parallelized region=======================================
        //stoneGrabber.armInComboAuto(true);
        stoneGrabber.armInCombo(true, true);
        if (Thread.currentThread().isInterrupted()) return 0;

        chassis.driveAuto(.5, (isBlue ? -6 : -10), 0, 1000);
        if (Thread.currentThread().isInterrupted()) return 0;
        if (Math.abs(chassis.getCurHeading()) > 1.0) {
            chassis.rotateTo(auto_chassis_align_power, 0, 500);
        }
        if (ss_pos == 1) {
            long iniTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - iniTime < 450) && !Thread.interrupted()) {
                TaskManager.processTasks();
            }
        } else if (ss_pos == 2) {
            long iniTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - iniTime < 250) && !Thread.interrupted()) {
                TaskManager.processTasks();
            }
        }
        if (!Thread.interrupted()) { //  wait a little bit for lifter to go down
            sleep(200);
        }
        // the following step crossing the bridge
        chassis.driveAuto(.8, 160 + 20 * ss_pos, -90 * side, 5000);
        if (Thread.currentThread().isInterrupted()) return 0;
        while (!TaskManager.isComplete("Arm In Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }
        //       chassis.rotateTo(auto_chassis_align_power, 0);
        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        int dForward = 0;
        if (safe && !Thread.interrupted()) {
            chassis.driveAuto(auto_chassis_power, -10, 0, 2000);
            dForward = 10;

            if (dist < 40) {
                dForward = findFoundtaion(isBlue);
                dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
            }
        }
        dist = Math.min(dist - 40, 55); // min for 55 cm as the range sensor is not accurate for long distance (over 50 cm) here.
        if (Thread.interrupted()) return 0;
        if (dist > 3) {
            chassis.driveAuto(auto_chassis_power, dist, -90 * side, 3000);
        }
        if (Thread.interrupted()) return 0;

        if (dForward != 0) {
            chassis.driveAuto(auto_chassis_power, dForward, 0, 2000);
        }
        if (Thread.currentThread().isInterrupted()) return 0;

        //=================================Parallelized region=======================================

        return ss_pos;
    }


    public int findFoundtaion(boolean isBlue) throws InterruptedException {
        int side = isBlue ? 1 : -1;
        int count = 0;
        while (chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT) < 40) {
            chassis.driveAuto(.8, -10, 0, 1000);
            count++;
            if (count > 6) {
                break;
            }
        }
        chassis.driveAuto(.8, -10, 0, 1000);
        return count * 10 + 30;
    }

    public void getAnotherSkyStoneNew(int ss_pos, int stoneNum, boolean isBlue, boolean placeSS) throws InterruptedException {//stoneNum - how many stones ara we going to have after this trip
        int side = isBlue ? 1 : -1;

        int toTake;
        //if (ss_pos == 3 && isBlue) {
        //  int[] a = {6, 1, 2, 4, 5};
        //toTake = a[stoneNum - 2];
        //} else
        if (ss_pos == 2 || (ss_pos == 3)) {
            int[] a = {5, 1, 3, 4, 6};
            toTake = a[stoneNum - 2];
        } else { // left or unknown
            int[] a = {4, 2, 3, 5, 6};
            toTake = a[stoneNum - 2];
        }
        //telemetry.addData("ssPos",ss_pos);
        //telemetry.addData("toTake",toTake);
        // stoneGrabber.armInComboAuto(false);// ASK CHARLIE
        // chassis.driveStraightAutoRunToPosition(.6, 55, 0, 1500);
        //chassis.rotateTo(auto_chassis_align_power, 0);

        if (Thread.interrupted()) return;
        stoneGrabber.armInCombo(false, true);
        chassis.rotateTo(.7, 0);
        sleep(100);
        double dist = Math.max(10, Math.min(70, chassis.getDistance(SwerveChassis.Direction.BACK)));
        if (isBlue) {
            chassis.driveAuto(.5, 60 - dist, 0, 1000);
        } else {
            chassis.driveAuto(.5, 55 - dist, 0, 1000);
        }
        chassis.rotateTo(auto_chassis_align_power, 0);
        if (Thread.interrupted()) return;
        if (toTake == 6) {
            stoneGrabber.armOutCombo(1, true);
            chassis.driveAuto(.8, 210, 90 * side, 5000);//<==============================
        } else if (toTake == 5) {
            stoneGrabber.armOutCombo(.85, true);
            chassis.driveAuto(.8, 190, 90 * side, 5000);
        } else {
            stoneGrabber.armOutCombo(.7, true);
            chassis.driveAuto(.8, 170, 90 * side, 5000);
        }
        stoneGrabber.grabberOpenAuto();
        if (Thread.interrupted()) return;
        chassis.rotateTo(auto_chassis_align_power, 0);
        if (Thread.interrupted()) return;
        while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }
        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.RIGHT : SwerveChassis.Direction.LEFT);
        //  telemetry.addData("distance", dist);
        //telemetry.addData("goLeft",-dist - 10 + 20 * (6 - toTake));
        //telemetry.addData("goLeft",telemetry.addData("goLeft",-dist - 10 + 20 * (6 - toTake)));
        //telemetry.update();
        int timeout = 1500;
        if (isBlue) {
            if (toTake != 6) {
                chassis.driveAuto(.40, -dist - 7 + 20.32 * (6 - toTake), -90, timeout);//was -5
            } else {
                chassis.driveAuto(.40, -dist, -90, timeout);
            }
        } else {
            if (toTake != 6) {
                chassis.driveAuto(.40, -dist - 15 + 20.32 * (6 - toTake), +90, timeout);
            } else {
                chassis.driveAuto(.40, -dist, +90, timeout);
            }
        }

        dist = (Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT)));
//        telemetry.addData("distance to stone",dist);
//        telemetry.update();
//        sleep(20000);
        if (Thread.interrupted()) return;
        if (Math.abs(dist - 10) > 1) {
            if (dist >= 11)
                stoneGrabber.armOutReadyGrabAutoCombo();
            chassis.driveAuto(.35, dist - 10, 0, 1500);
        }
//        if (dist - 14 > 1) {
//            chassis.driveAuto(.35, dist - 14, 0, 1000);////?
//        } else if (dist - 14 < -1) {
//            chassis.driveAuto(.35, dist - 14, 0, 1000);////?
//        }
        if (toTake == 6) {
            chassis.rotateTo(.4, 10 * side, 50);
        }
        if (Thread.interrupted()) return;
        stoneGrabber.grabStoneComboAutoHigher();//<================grab the second ss
        if (Thread.interrupted()) return;
        if (toTake == 6) {
            chassis.driveAuto(.8, 20, -90 * side, 1000);
        }
        if (Thread.interrupted()) return;
        chassis.rotateTo(.65, (isBlue ? -90 : 89.5), 2000);
        if (Thread.interrupted()) return;
        stoneGrabber.lifterDownCombo();
        if (Thread.interrupted()) return;
        // foundationHook.hookDown();
        //stoneGrabber.grabberOpen();

        if (toTake != 6) {
            chassis.driveAuto(.8, (placeSS ? 200 : 140) + 20.32 * (toTake - 3), 0, 7000);
        } else {
            chassis.driveAuto(.8, (placeSS ? 205 : 140) + 20.32 * (toTake - 3), 0, 7000);
        }

        if (placeSS && !Thread.interrupted()) {
            sleep(200);
            double disToFoundation = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT), chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT));
//            telemetry.addData("dis to foundation",disToFoundation);
//            telemetry.update();
//            sleep(2000);
            disToFoundation = Math.min(disToFoundation, 30);
            disToFoundation = Math.max(disToFoundation, 15);

            chassis.driveAuto(0.35, disToFoundation - 2, 0, 1000);
        }
        if (Thread.interrupted()) return;

        stoneGrabber.grabberOpenAuto();

        chassis.driveAuto(0.5, 10, 0, 500);
        if (Thread.interrupted()) return;
        chassis.rotateTo(auto_chassis_align_power, isBlue ? -90 : 90, 500);
        //chassis.driveStraightAutoRunToPosition(.9, -40, 0, 15000);

    }

    public void park2SS(boolean placing) throws InterruptedException {
        chassis.driveAuto(.9, placing ? -100 : -40, 0, 3000);
        chassis.driveAuto(.4, -20, 0, 1000);
    }

    public void rotateFoundation(boolean isBlue) throws InterruptedException {//NEW ProgrAM
        //if (Thread.interrupted()) return;
        int side = isBlue ? 1 : -1;
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT));
        chassis.driveAuto(0.4, dist + 5, 0, 1000);//isBlue ? 23 : 28
        foundationHook.hookDown();
        //if (Thread.interrupted()) return;
        chassis.driveStraightSec(0.25, 0.2, false);
        //if (Thread.interrupted()) return;

        stoneGrabber.armOutCombo();
        chassis.driveAuto(.9, -28, -50 * side, 1500);//50?
        /*while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }*/
        //stoneGrabber.wristPerpendicular();
        //if (Thread.interrupted()) return;
        chassis.changeStopBehavior(false);
        chassis.driveStraightSec(-.7, 0.05, true);//<======gain momentum
        if (Thread.interrupted()) return;
        stoneGrabber.deliverStoneCombo(true);
//        chassis.rawRotateTo(.80, 85 * side, true, 2000);
        chassis.rotateTo(0.9, 110 * side, 1700, false, false); // was 85 and 2000
        //stoneGrabber.deliverStoneCombo(true);
        if (Thread.interrupted()) return;
        chassis.changeStopBehavior(true);
        chassis.driveAuto(.9, 30, 0, 1000);
        if (Thread.interrupted()) return;
        foundationHook.hookUp();
        chassis.driveAuto(.9, -10, 0, 1500);
        if (Thread.interrupted()) return;
        stoneGrabber.armInReadyGrabCombo();
        //stoneGrabber.grabberClose();
    }

    public void rotateFoundationNew(boolean isBlue) throws InterruptedException {
        if (Thread.interrupted()) return;
        int side = isBlue ? 1 : -1;
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT));
        chassis.driveAuto(auto_chassis_power, dist + 5, 0, 1000);//isBlue ? 23 : 28
        foundationHook.hookDown();
        if (Thread.interrupted()) return;
        chassis.driveStraightSec(0.25, 0.2, false);
        if (Thread.interrupted()) return;

        stoneGrabber.armOutCombo();
        chassis.driveAuto(.9, -32, -55 * side, 1500);//50?
        /*while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }*/
        //stoneGrabber.wristPerpendicular();
        if (Thread.interrupted()) return;
        chassis.changeStopBehavior(false);
        chassis.driveStraightSec(-.9, (isBlue ? 0.2 : .15), true);
        if (Thread.interrupted()) return;
        stoneGrabber.deliverStoneCombo(true);
        chassis.rawRotateTo(.95, -80 * side, true, 2000);
        //stoneGrabber.deliverStoneCombo(true);
        if (Thread.interrupted()) return;
        chassis.changeStopBehavior(true);
        chassis.driveAuto(.9, 30, 0, 1000);
        if (Thread.interrupted()) return;
        foundationHook.hookUp();
        chassis.driveAuto(.9, -10, 0, 1500);
        if (Thread.interrupted()) return;
        stoneGrabber.armInCombo(false, true);
        stoneGrabber.grabberClose();


        //while (!TaskManager.isComplete("Deliver Stone Combo")&&!Thread.interrupted()) {
        //  TaskManager.processTasks();
        //}

    }

    public void rotateFoundationOnly(boolean isBlue, boolean l2) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        double dist = Math.min(chassis.getDistance(SwerveChassis.Direction.FRONT_RIGHT), chassis.getDistance(SwerveChassis.Direction.FRONT_LEFT));
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, dist + 5, 0, 1000);//isBlue ? 23 : 28
        foundationHook.hookDown();
        chassis.driveStraightSec(0.25, 0.2, false);
        chassis.driveStraightAutoRunToPositionNoIMU(.8, -50, -50 * side, 1500);//.7?
        chassis.changeStopBehavior(false);
        chassis.driveStraightSec(-.7, .2, true);
        chassis.rawRotateTo(.8, -85 * side, true, 3000);
        chassis.changeStopBehavior(true);
        chassis.driveStraightAutoRunToPosition(.9, 30, 0, 1000);
        foundationHook.hookUp();
        chassis.driveStraightAutoRunToPosition(.9, -10, 0, 1500);

        dist = Math.min(75, chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT));
        if (l2) {
            chassis.driveStraightAutoRunToPosition(.6, dist, -90 * side, 5000);
        } else {
            chassis.driveStraightAutoRunToPosition(.6, dist - 65, -90 * side, 5000);
        }
        chassis.driveStraightAutoRunToPosition(.6, -80, 0, 5000);


    }

    public void parkAfterRotateNew(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        chassis.driveAuto(.4, 85, side * 90, 3000);

    }

    public void getWallStone(boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int negtiveDegree = isBlue ? -1 : 1;
        if (Thread.currentThread().isInterrupted()) return;
        stoneGrabber.armInReadyGrabCombo();
        if (Thread.currentThread().isInterrupted()) return;

        intake.intakeDropDown();
        intake.ingateOpen();
        chassis.rotateTo(.22, negtiveDegree * 90);
        if (Thread.currentThread().isInterrupted()) return;

        double dist = chassis.getDistance(negtiveDegree == 1 ? SwerveChassis.Direction.RIGHT : SwerveChassis.Direction.LEFT);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.6, -dist + (isBlue ? 65 : 58), negtiveDegree * (-90), 2000);
        if (Thread.currentThread().isInterrupted()) return;
        double currHeading = chassis.getCurHeading();
        if (isBlue) {
            if (currHeading < -90.5 || currHeading > -88)
                chassis.rotateTo(.20, -90);//was -89.5
        } else {
            if (currHeading > 90.5 || currHeading < 88)
                chassis.rotateTo(.20, 89);
        }
        if (Thread.currentThread().isInterrupted()) return;

        chassis.driveAuto(.9, -250, 0, 5000);//drive all the way to the wall
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.4, -10, 0, 500);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.rotateTo(.22, negtiveDegree * 90);
        //dist = chassis.getDistance(negtiveDegree == 1 ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        //chassis.driveAuto(.3, dist - 15, negtiveDegree * (-90), 2000);//adjust distance to stone
        if (Thread.currentThread().isInterrupted()) return;
        sleep(100);


        //chassis.driveAuto(.3, -dist + 25, 0, 2000);//adjust distant to back wall
        if (Thread.currentThread().isInterrupted()) return;

        chassis.rotateTo(.5, negtiveDegree * 135);
        if (Thread.currentThread().isInterrupted()) return;
        intake.intakeIn(true);
        chassis.driveAuto(.5, -40, 0, 2000);//suck in the stone
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.8, 30, 0, 2000);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.rotateTo(.5, (isBlue ? -90 : 91));//was 90.5
        if (Thread.currentThread().isInterrupted()) return;
        intake.intakeStop();
        intake.ingateClose();
        //stoneGrabber.grabStoneInsideCombo();
        //chassis.rotateTo(.3 90);
        stoneGrabber.grabInsideAndArmOutCombo(.75, true);
        if (Thread.interrupted()) return;
        chassis.driveAuto(.85, 240, 0, 5000);//drive all the way to the foundation
        if (Thread.interrupted()) return;
        while (!TaskManager.isComplete("Grab Inside and Arm Out Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }
        chassis.driveAuto(.3, 10, 0, 400);
        if (Thread.interrupted()) return;

        chassis.rotateTo(0.3, negtiveDegree * 95, 400);//fine adjustment
        if (Thread.interrupted()) return;
        stoneGrabber.grabberOpenAuto();//place skystone
        sleep(200);
        chassis.rotateTo(0.22, isBlue ? -90.5 : 88, 500);//fine adjustment
        if (Thread.interrupted()) return;
    }

    public void align(double distBack, double distLeft, double desiredDistBack, double desiredDistLeft) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        double dx = desiredDistBack - distBack; // vertical
        double dy = desiredDistLeft - distLeft;// horizontal
//        telemetry.addLine("ddb: " + desiredDistBack + "db" + distBack + "ddl" + desiredDistLeft + "dl" + distLeft);
//        telemetry.update();
        //sleep(7000);
        if (Thread.currentThread().isInterrupted()) return;
        CDist move = diagonalMove(dx, dy);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.4, move.distance, move.angle, 3000);

    }

    public CDist diagonalMove(double dx, double dy) throws InterruptedException {
        CDist move = new CDist();
        int s = 1;
        //telemetry.addLine("dx:" + dx + "  dy:" + dy);
        //info("dx:" + dx + "  dy:" + dy);
        //telemetry.update();
        //sleep(5000);
        if (dx > 0 && dy > 0) {
            move.angle = 90 - Math.atan(dx / dy) / Math.PI * 180;
        } else if (dx > 0 && dy <= 0) {
            move.angle = -90 + Math.atan(Math.abs(dx / dy)) / Math.PI * 180;
        } else if (dx < 0 && dy > 0) {
            move.angle = -90 + Math.atan(Math.abs(dx / dy)) / Math.PI * 180;
            s = -1;
        } else {
            move.angle = 90 - Math.atan(Math.abs(dx / dy)) / Math.PI * 180;
            s = -1;
        }
        move.distance = Math.hypot(dx, dy) * s;
        //telemetry.addLine("pow: " + .4 + "  dist: " + move.distance + " angle: " + move.angle);
        //telemetry.update();
        info("pow: " + .4 + "  dist: " + move.distance + " angle: " + move.angle);
        return move;
    }

    public void wheelIntakeSecondStone(int stoneNum, int ss_pos, boolean isBlue) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
//        int negtiveDegree = isBlue ? -1 : 1;
        int toTake;
        if (ss_pos == 3) {
            int[] a = {6, 1, 2, 4, 5};
            toTake = a[stoneNum - 2];
        } else if (ss_pos == 2) {
            int[] a = {5, 1, 3, 4, 6};
            toTake = a[stoneNum - 2];
        } else {
            int[] a = {4, 2, 3, 5, 6};
            toTake = a[stoneNum - 2];
        }
        if (Thread.currentThread().isInterrupted()) return;
        intake.intakeDropDown();
        intake.ingateOpen();
        chassis.rotateTo(.22, isBlue ? +90 : -90);
        if (Thread.currentThread().isInterrupted()) return;
        double dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT_HI : SwerveChassis.Direction.RIGHT_HI);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.6, -dist + (isBlue ? 62 : 58), isBlue ? +90 : -90, 2000);
        if (Thread.currentThread().isInterrupted()) return;
        double currHeading = chassis.getCurHeading();
       /* if (isBlue) {
            if (currHeading < -90.5 || currHeading > -88)
                chassis.rotateTo(.20, -90);//was -89.5
        } else {
            if (currHeading > 90.5 || currHeading < 88)
                chassis.rotateTo(.20, 89);
        }*/
        chassis.rotateTo(.20, isBlue ? +90 : -90);

        if (Thread.currentThread().isInterrupted()) return;
        if (toTake == 6) {
            chassis.driveAuto(.90, -200, 0, 5000);//                          changeeeeeeeeeeeee

        } else if (toTake == 5) {
            chassis.driveAuto(.90, -190, 0, 5000);//                          changeeeeeeeeeeeee
        } else {
            chassis.driveAuto(.90, -180, 0, 5000);//                          changeeeeeeeeeeeee

        }

        if (Thread.currentThread().isInterrupted()) return;
//        chassis.driveAuto(.4, -10, 0, 500);
        //bold move
//        dist = chassis.getDistance(SwerveChassis.Direction.BACK);
//        if (dist > 15)
//            chassis.driveAuto(.5, -dist + 15, 0, 2000);//adjust distant to back wall
        if (Thread.currentThread().isInterrupted()) return;
        chassis.rotateTo(.22, isBlue ? +90 : -90, 1000);
        /*
        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.RIGHT : SwerveChassis.Direction.LEFT);
        chassis.driveAuto(.5, dist - 12, isBlue ? +90 : -90, 2000);//adjust distance to stone  was 12
        */

        if (Thread.currentThread().isInterrupted()) return;
        double distBack = chassis.getDistance(SwerveChassis.Direction.BACK);
        double distLeft = side * chassis.getDistance(side == 1 ? SwerveChassis.Direction.LEFT_HI : SwerveChassis.Direction.RIGHT_HI);
        align(distBack, distLeft, isBlue ? (27 + (6 - toTake) * 20.3) : (29 + (6 - toTake) * 20.3), isBlue ? 78 : -71);
        //dist = chassis.getDistance(SwerveChassis.Direction.BACK);
        //chassis.driveAuto(.5, -dist + (isBlue? 27:29) + (6 - toTake) * 20.3, 0, 2000);//adjust distant to back wall

        if (Thread.currentThread().isInterrupted()) return;
        sleep(100);

        //dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT_HI : SwerveChassis.Direction.RIGHT_HI);
        //chassis.driveAuto(.4, isBlue ? 73 -dist : 71 -dist, isBlue ? +90 : -90, 1000);// changeeeeeeeeeeeeeeeeeeeeeeeeedddddddddddd
        if (Thread.currentThread().isInterrupted()) return;

        chassis.rotateTo(.5, isBlue ? +45 : -45);
        if (Thread.currentThread().isInterrupted()) return;
        intakeInAuto(true);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.4, isBlue ? -35 : -33, 0, 2000);//suck in the stone
        //******wait till stone is in
        long iniTime = System.currentTimeMillis();
        while (!chassis.stoneCollected() && (System.currentTimeMillis() - iniTime) < 800) {
            if (Thread.interrupted()) return;
        }
        if (Thread.currentThread().isInterrupted()) return;
        intake.ingateClose();
        if (Thread.currentThread().isInterrupted()) return;
        intake.intakeStop();
        //******
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.9, isBlue ? 30 : 41, 0, 2000);//was 30
        if (Thread.currentThread().isInterrupted()) return;
        chassis.rotateTo(.5, (isBlue ? 90 : -88));//was 90.5
        if (Thread.currentThread().isInterrupted()) return;
//        dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
//        chassis.driveAuto(.4, -dist + 56, 90* side, 1000);
//        intake.intakeStop();
//        intake.ingateClose();
        //stoneGrabber.grabStoneInsideCombo();
        //chassis.rotateTo(.3 90);

    }

    public void deliverAndPark2SS(boolean isBlue, int ss_pos) throws InterruptedException {
        int side = isBlue ? 1 : -1;
        int toTake = ss_pos + 3;
        double dist = chassis.getDistance(side == 1 ? SwerveChassis.Direction.LEFT_HI : SwerveChassis.Direction.RIGHT_HI);

        boolean lowTime = getAutoTimeLeft() < 3;
        if (toTake == 6) {
            stoneGrabber.grabInsideAndArmOutCombo(.95, true);
            if (Thread.interrupted()) return;
            //chassis.driveAuto(.9, 260, 0, 5000);//drive all the way to the foundation
            CDist move = diagonalMove(lowTime ? 200 : 260, side * (-dist + 70));
            if (Thread.currentThread().isInterrupted()) return;
            chassis.driveAuto(.9, move.distance, Math.max(Math.min(move.angle, 3), -3), 4000);
        } else if (toTake == 5) {
            stoneGrabber.grabInsideAndArmOutCombo(.75, true);
            if (Thread.interrupted()) return;
            //chassis.driveAuto(.9, 240, 0, 5000);//drive all the way to the foundation
            CDist move = diagonalMove(lowTime ? 180 : 240, side * (-dist + 70));
            if (Thread.currentThread().isInterrupted()) return;
            chassis.driveAuto(.9, move.distance, Math.max(Math.min(move.angle, 3), -3), 4000);

        } else {
            stoneGrabber.grabInsideAndArmOutCombo(.65, true);
            if (Thread.interrupted()) return;
            CDist move = diagonalMove(lowTime ? 160 : 220, side * (-dist + 70));
            if (Thread.currentThread().isInterrupted()) return;
            chassis.driveAuto(.9, move.distance, Math.max(Math.min(move.angle, 3), -3), 4000);

            //chassis.driveAuto(.9, 220, 0, 5000);//drive all the way to the foundation
        }
        if (Thread.interrupted()) return;
        while (!TaskManager.isComplete("Grab Inside and Arm Out Combo") && !Thread.interrupted()) {
            TaskManager.processTasks();
        }
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.5, 10, 0, 200);

        if (lowTime){
            stoneGrabber.grabberOpen();
            if (Thread.interrupted()) return;
            stoneGrabber.lifterDownCombo();
            if (Thread.interrupted()) return;
            chassis.driveAuto(0.9,-65,0,1500);
            return;
        }
//        chassis.rotateTo(0.3, negtiveDegree * -95, 400);//fine adjustment
        if (Thread.interrupted()) return;
        // stoneGrabber.grabberOpenAuto();//place skystone
        //sleep(100);
        stoneGrabber.deliverStoneThrowComboAuto();
        if (Thread.currentThread().isInterrupted()) return;
        chassis.driveAuto(.6, -10, 0, 500);
        if (Thread.currentThread().isInterrupted()) return;
        chassis.rotateTo(0.22, isBlue ? 89 : -91, 500);//fine adjustment

        stoneGrabber.lifterDownCombo();
        if (Thread.interrupted()) return;
        dist = chassis.getDistance(side == 1 ? SwerveChassis.Direction.LEFT_HI : SwerveChassis.Direction.RIGHT_HI);
        CDist move = diagonalMove(-110, side * (-dist + 65));
        chassis.driveAuto(.9, move.distance, Math.max(Math.min(move.angle, 10), -10), 4000);
        //stoneGrabber.lifterDownCombo(0.5);

        //stoneGrabber.armInCombo(true,true);
        //chassis.driveAuto(.7, -110, 5, 5000);
    }

    public void rotateFoundation(boolean isBlue, boolean moveArm) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;
        int side = isBlue ? 1 : -1;
        if (moveArm)
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, isBlue ? 23 : 28, 0, 1000);
        foundationHook.hookDown();
        chassis.driveStraightSec(0.25, 0.5, false);
        if (Thread.interrupted()) return;
        if (moveArm)
            stoneGrabber.armOutCombo();
        chassis.driveStraightAutoRunToPositionNoIMU(.8, -50, -50 * side, 1500);//.7?
        /*while (!TaskManager.isComplete("Arm Out Combo")&&!Thread.interrupted()) {
            TaskManager.processTasks();
        }*/
        //stoneGrabber.wristPerpendicular();
        if (Thread.interrupted()) return;
        chassis.changeStopBehavior(false);
        chassis.driveStraightSec(-.7, .2, true);
        if (moveArm)
            stoneGrabber.deliverStoneCombo(true);
        if (Thread.interrupted()) return;
        chassis.rawRotateTo(.8, -85 * side, true, 3000);
        //stoneGrabber.deliverStoneCombo(true);
        if (Thread.interrupted()) return;
        chassis.changeStopBehavior(true);
        chassis.driveStraightAuto(.6, 35, 0, 1000);
        if (Thread.interrupted()) return;
        foundationHook.hookUp();
        chassis.driveStraightAuto(auto_chassis_power, -10, 0, 1500);
        if (Thread.interrupted()) return;
        if (moveArm) {
            stoneGrabber.armInCombo(false, true);
            if (Thread.interrupted()) return;
            chassis.rotateTo(.5, 0);
            double dist = Math.max(10, Math.min(70, chassis.getDistance(SwerveChassis.Direction.BACK)));
            if (Thread.interrupted()) return;
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 53 - dist, 0, 1500);
            while (!TaskManager.isComplete("Deliver Stone Combo") && !Thread.interrupted()) {
                TaskManager.processTasks();
            }
        }

    }

    public void autoBackupProgram(boolean isBlue, boolean laneFront, boolean offensive, boolean parkOnly) throws InterruptedException {
        if (Thread.interrupted()) return;
        int side = isBlue ? 1 : -1;
        if (parkOnly) {
            chassis.driveStraightAutoRunToPosition(.4, 82, 90 * side, 5000);
            if (laneFront) {
                chassis.driveStraightAutoRunToPosition(.4, 60, 0, 4000);
            }
            return;
        }
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 10, -90 * side, 1500);
        if (Thread.interrupted()) return;
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 70, 0, 1500);
        if (Thread.interrupted()) return;
        if (offensive) {
            foundationHook.hookDown();
            if (Thread.interrupted()) return;
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 70, 0, 1500);
            if (Thread.interrupted()) return;
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, -70, 0, 1500);
            if (Thread.interrupted()) return;
            double dist2 = chassis.getDistance(SwerveChassis.Direction.BACK);
            if (dist2 > 128) dist2 = 70;
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 70 - dist2, 0, 1500);
            if (Thread.interrupted()) return;
            chassis.rotateTo(.6, 0);
            if (Thread.interrupted()) return;
        }

        rotateFoundation(isBlue, false);
        if (Thread.interrupted()) return;
        chassis.rotateTo(0.25, -90 * side);
        if (Thread.interrupted()) return;
        double dist = chassis.getDistance(isBlue ? SwerveChassis.Direction.LEFT : SwerveChassis.Direction.RIGHT);
        if (dist > 45) dist = 45;
        if (laneFront) {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 60 - dist, 90 * side, 1500);
        } else {
            chassis.driveStraightAutoRunToPosition(auto_chassis_power, 10 - dist, 90 * side, 1500);
        }
        if (Thread.interrupted()) return;
        chassis.driveStraightAutoRunToPositionNoIMU(.4, -85, 0, 3000);
        if (Thread.interrupted()) return;
    }

    public void getFoundation(boolean isBlue, boolean laneTwo) throws InterruptedException {
        int side = isBlue ? 1 : -1;
        chassis.driveStraightAutoRunToPosition(auto_chassis_power, 70, 0, 1500);
        rotateFoundationOnly(isBlue, laneTwo);
    }

    public void setupTelemetryTensor(Telemetry telemetry) {
        if (Thread.currentThread().isInterrupted()) return;
        Telemetry.Line line = telemetry.addLine();
        line.addData(" | <A> Iterations", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n", tensorPara.getIter());
            }
        });
        line.addData("| <B> Correct Stone Config", new Func<String>() {
            public String value() {
                return String.format("%s\n", tensorPara.correctLoc() == 1 ? "LEFT" : tensorPara.correctLoc() == 2 ? "CENTER" : "RIGHT");
            }
        });
    }

    public void tensorTest(int iter, int loc)//loc = 1 left, 2 center, 3 right
    {
        int normalsizestonecount = 0;
        int bigstoneleftcount = 0;
        int bigstonerightcount = 0;
        int correctstonecount = 0;
        double[][] testresults = new double[iter][2];
        double stoneloc;
        double[] sslocation = new double[2];

        for (int i = 0; i < iter; i++) {
            cameraStoneDetector.SSLocTest(sslocation);
            for (int j = 0; i < 2; j++) {
                testresults[i][j] = sslocation[j];
            }
        }
        for (int i = 0; i < iter; i++) {
            if (testresults[i][1] - testresults[i][0] <= 250) {
                normalsizestonecount++;
                correctstonecount = (((Math.round(((testresults[i][0] + testresults[i][1]) / 2) / 200)) * 200) == loc) ? correctstonecount++ : correctstonecount;
            } else if (testresults[i][1] - testresults[i][0] > 250) {
                if ((testresults[i][0] + testresults[i][1]) / 2 < 200) {
                    bigstoneleftcount++;
                } else if ((testresults[i][0] + testresults[i][1]) / 2 > 400) {
                    bigstonerightcount++;
                }
            }
        }

        telemetry.addLine().addData("TFOD:", "Normal/Correct=%1d/%1d, BigL=%1d,BigR=%1d",
                normalsizestonecount, correctstonecount, bigstoneleftcount, bigstonerightcount).setRetained(true);
        telemetry.update();

    }

    @MenuEntry(label = "Tensor Test", group = "Test Auto")
    public void tensorTestFunc(EventManager em, Configuration configuration) {
        telemetry.addLine().addData(" | <X>", "Done").setRetained(true);
        setupTelemetryTensor(telemetry);

        cameraStoneDetector = new CameraStoneDetector().configureLogging("CameraStoneDetector", logLevel);
        cameraStoneDetector.configure(configuration, CameraSource.WEBCAM_RIGHT);

        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                tensorPara.iter = tensorPara.iter <= 4 ? tensorPara.iter++ : 1;
            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                tensorPara.correctloc = tensorPara.correctloc < 3 ? tensorPara.correctloc++ : 1;
            }
        }, new Button[]{Button.B});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                //tensorTest(tensorPara.getIter(), tensorPara.correctLoc());
                simpleGetStoneTest();
            }
        }, new Button[]{Button.X});
    }


    public void simpleGetStoneTest() {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
        while (elapsedTime.seconds() < 0.3) {
            List<Recognition> updatedRecognitions = cameraStoneDetector.getTfod().getUpdatedRecognitions();
            int i = 0;
            for (Recognition recognition :
                    updatedRecognitions) {
                i++;
                double width = recognition.getWidth();
                double height = recognition.getHeight();
                double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                telemetry.addLine().addData("Stone", "%d:: wid=%.1f,he=%.1f,ang=%.1f", i, width, height, angle).setRetained(true);
                telemetry.update();

            }
        }
    }

}
