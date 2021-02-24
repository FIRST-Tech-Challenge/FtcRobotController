package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import static java.lang.Thread.interrupted;

/**
 * FoundationHook spec:
 */
public class ComboGrabber extends Logger<ComboGrabber> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo arm;
    private DcMotorEx slider;
    private AdjustableServo grabber;

    private final int grabberVersion = 1; // current version used for LeagueMeet 0
                                         // 2 is new grabber
    private final double SLIDER_POWER = 0.5;
    private final double SLIDER_SPEED = 2500;
    private final int SLIDER_POS_HIGH = 920;
    private final int SLIDER_POS_HIGHER = 1500;
    private final int SLIDER_POS_INIT = 0;
    private final int SLIDER_POS_LOW = 0;
    private final int SLIDER_POS_MAX = 1733;
    private final int SLIDER_POS_RING = 580;

    private double ARM_UP = 0.4;
    private double ARM_INIT = 0.16;
    private double ARM_DOWN_RELEASE = 0.91;
    private double ARM_DOWN = 0.95;
    private double ARM_COLLECT_RING = 0.53;

    private double GRABBER_OPEN = 0.35;
    private double GRABBER_CLOSE = 0.9;
    private double GRABBER_PARTIAL_CLOSE = 0.8;
    private double GRABBER_INIT = GRABBER_CLOSE;

    private boolean sliderIsLow = true;
    private boolean armIsLow = false;
    private boolean grabberIsClosed = false;
    private  boolean isGrabFromBottom = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "topWobbleGoalGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public ComboGrabber(CoreSystem core) {
        this.core = core;
    }

    public boolean isArmLow() {
        return armIsLow;
    }

    public void reset(boolean Auto) {
        servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        if (grabberVersion ==  2) {
            ARM_UP = 0.48;
            ARM_INIT = 0.16;
            ARM_DOWN_RELEASE = 0.91;
            ARM_DOWN = 0.95;
            ARM_COLLECT_RING = 0.53;

            GRABBER_OPEN = 0.52;
            GRABBER_CLOSE = 0.9;
            GRABBER_INIT = GRABBER_CLOSE;
        }
        grabber = new AdjustableServo(0, 1).configureLogging(
                logTag + ":topWobbleGoalGrabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);

        slider = configuration.getHardwareMap().get(DcMotorEx.class, "slider");
        if (slider != null) {
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        arm = new AdjustableServo(0, 1).configureLogging(
                logTag + ":topWobbleGoalGrabber", logLevel
        );
        arm.configure(configuration.getHardwareMap(), "arm");
        configuration.register(arm);
        configuration.register(this);
        // servoInit();
    }

    public void servoInit() {
        if (grabber!=null)
            grabber.setPosition(GRABBER_INIT);
        if (arm!=null)
            arm.setPosition(ARM_INIT);
        sliderIsLow = false;
        grabberIsClosed = true;
        // configuration.register(this);
    }

    public void sliderStop() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(0);
    }

    public Progress slideToPos(int pos) {
        if (slider==null) return null;
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setVelocity(SLIDER_SPEED);
        return new Progress() {
            public boolean isDone() {
                if (Math.abs(slider.getCurrentPosition() - slider.getTargetPositionTolerance()) < 50) {
                    return true;
                }
                return !slider.isBusy();
            }
        };
    }

    public void sliderPosHigh() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_HIGH);
        sliderIsLow = false;
    }

    public void slidePosLow() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_LOW);
        sliderIsLow = true;
    }

    public void sliderPosInit() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_INIT);
        sliderIsLow = false;
    }

    public void sliderPosMax() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_MAX);
        sliderIsLow = false;
    }

    public void sliderPosAuto() {
        if (slider==null) return;
        if (sliderIsLow)
            sliderPosHigh();
        else
            slidePosLow();
    }

    public void sliderUp(boolean forced) {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = slider.getCurrentPosition();
        if ((pos>=SLIDER_POS_MAX) && !forced) {
            sliderStop();
            return;
        }
        pos = Math.min(pos+100, SLIDER_POS_MAX);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setVelocity(SLIDER_SPEED);
        // armMotor.setPower(ARM_POWER);
        // armIsDown = false;
    }

    public void sliderDown(boolean forced) {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = slider.getCurrentPosition();
        if (pos<=SLIDER_POS_INIT && !forced) {
            sliderStop();
            return;
        }
        pos=Math.max(pos-100,SLIDER_POS_INIT);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setVelocity(-SLIDER_SPEED);
        // armMotor.setPower(-ARM_POWER);
        // armIsDown = true;
    }

    public void armUpInc() {
        if (arm==null) return;
        double pos=arm.getPosition()-0.05;
        if (pos<0) pos=0;
        arm.setPosition(pos);
        if (pos<ARM_UP+0.1)
            armIsLow=false;
    }

    public void armDownInc() {
        if (arm==null) return;
        double pos=arm.getPosition()+0.05;
        if (pos>1) pos=1.0;
        arm.setPosition(pos);
        if (pos>ARM_DOWN-0.1)
            armIsLow=true;
    }

    public void armUp() {
        if (arm==null) return;
        arm.setPosition(ARM_UP);
        armIsLow = false;
    }

    public void armDown() {
        if (arm==null) return;
        arm.setPosition(ARM_DOWN);
        armIsLow = true;
    }

    public void armAuto() {
        if (armIsLow) {
            armUp();
        } else {
            armDown();
        }
    }

    public void grabberOpen(){
        grabber.setPosition(GRABBER_OPEN);
        grabberIsClosed = false;
    }

    public void grabberClose(){
        grabber.setPosition(GRABBER_CLOSE);
        grabberIsClosed = true;
    }
    public void grabberPartialClose(){
        grabber.setPosition(GRABBER_PARTIAL_CLOSE);
        grabberIsClosed = true;
    }
    public void grabberAuto(){
        if (grabberIsClosed) {
            grabberOpen();
        } else {
            grabberClose();
        }
    }
    public void releaseWobbleGoalCombo() {
        final String taskName = "release Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
//        if (slider.getCurrentPosition()>SLIDER_POS_HIGH) {
//            TaskManager.add(new Task() {
//                @Override
//                public Progress start() {
//                    return slideToPos(SLIDER_POS_HIGH);
//                }}, taskName);
//        }
        if (isGrabFromBottom) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_INIT);
                }}, taskName);
        }

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();return moveArm(ARM_DOWN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_UP);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_PARTIAL_CLOSE);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_INIT);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return slideToPos(SLIDER_POS_INIT);
            }}, taskName);
    }

    public void releaseWobbleGoalFastCombo() {
        final String taskName = "release Wobble Goal Fast Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (slider!=null && slider.getCurrentPosition()>SLIDER_POS_HIGH) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_HIGH);
                }}, taskName);
        }
        if (slider != null && isGrabFromBottom) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_INIT);
                }}, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_RELEASE);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }}, taskName);
    }

    public void initWobbleGoalCombo() { // put wobble goal to init position
        final String taskName = "init Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        grabberPartialClose();
        moveArm(ARM_INIT);
        if(slider!=null)
            TaskManager.add(new Task() {
                @Override
                public Progress start() { return slideToPos(SLIDER_POS_INIT);
                }}, taskName);
    }

    public void collectRingCombo(){
        grabWobbleGoalCombo(false);
        while (!TaskManager.isComplete("grab Wobble Goal Combo") && !interrupted()) {
            TaskManager.processTasks();
        }
        moveArm(ARM_COLLECT_RING);
        if (slider!=null)
            slideToPos(SLIDER_POS_RING);
    }

    public void grabWobbleGoalCombo(boolean isHigh) {
        final String taskName = "grab Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        isGrabFromBottom = !isHigh;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN);
            }}, taskName);
        // if (isHigh) {
        if (false) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_HIGH);
                }}, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_LOW);
                }}, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_CLOSE);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_UP);
            }}, taskName);
        if (isHigh) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_HIGHER);
                }}, taskName);
        }
    }

    public void grabWobbleGoalFastCombo() {
        final String taskName = "grab Wobble Goal Fast Combo";
        if (!TaskManager.isComplete(taskName)) return;
        isGrabFromBottom = false;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN);
            }}, taskName);
//        TaskManager.add(new Task() {
//                @Override
//                public Progress start() {
//                    return slideToPos(SLIDER_POS_HIGH);
//                }}, taskName);
        TaskManager.add(new Task() {
            @Override public Progress start() {
                return moveGrabber(GRABBER_CLOSE);
            }}, taskName);
    }

    public void raiseWobbleGoalCombo() {
        final String taskName = "raise Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_UP);
            }}, taskName);
        TaskManager.add(new Task() {
                @Override
                public Progress start() { return slideToPos(SLIDER_POS_HIGHER);
                }}, taskName);
    }

    private Progress moveArm(double position) {
        double adjustment = Math.abs(position - arm.getPosition());
        arm.setPosition(position);
        if (position>= ARM_DOWN - 0.3)
            armIsLow=true;
        else {
            armIsLow = false;
        }
        // 1000 ms per 180 degree
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 600);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    private Progress moveGrabber(double position) {
        double adjustment = Math.abs(position - grabber.getPosition());
        grabber.setPosition(position);
        if (position<= GRABBER_CLOSE + 0.1)
            grabberIsClosed=true;
        else {
            grabberIsClosed = false;
        }
        // 1200 ms per 180 degree
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 600);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }
    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (grabber != null) {
            line.addData("T-Grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }

        if (arm != null) {
            line.addData("Arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return arm.getPosition();
                }
            });
        }

        if (slider != null) {
            line.addData("slider", "%s", new Func<String>() {
                @Override
                public String value() {
                    String s = String.format("pos=%d, pw=%.1f, speed=%.1f", slider.getCurrentPosition(), slider.getPower(),
                            slider.getVelocity());
                    return s;
                }
            });
        }
    }

}



