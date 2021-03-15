package org.firstinspires.ftc.teamcode.hardware.Beta;

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
public class Grabber extends Logger<Grabber> implements Configurable {

    final private CoreSystem core;

    // private AdjustableServo arm;
    private DcMotorEx arm;
    private AdjustableServo grabber;

    private final int grabberVersion = 1; // current version used for LeagueMeet 0
                                         // 2 is new grabber
    private final double ARM_POWER = 0.5;
    private final double ARM_SPEED = 2500;
    private final double ARM_SPEED_SLOW = 2000;
    private int ARM_POS_MIN = 0;
    private int ARM_POS_MAX = 1300;
    private int ARM_POS_INIT = 0;
    private int ARM_POS_UP_AUTO = 250;
    private int ARM_POS_UP = 190;
    private int ARM_POS_UP_UP = 50;
    private int ARM_POS_DROP_HIGH = 500;
    private int ARM_POS_DROP = 950;
    private int ARM_POS_DOWN = 1050;
    private int ARM_POS_DOWN_DOWN = 1190;
    private int ARM_UNIT = 50;

    private double GRABBER_OPEN = 0.82;
    private double GRABBER_CLOSE = 0.5;
    private double GRABBER_PARTIAL_CLOSE = 0.56;
    private double GRABBER_INIT = GRABBER_CLOSE;

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
    public Grabber(CoreSystem core) {
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
            GRABBER_OPEN = 0.52;
            GRABBER_CLOSE = 0.9;
            GRABBER_INIT = GRABBER_CLOSE;
        }
        grabber = new AdjustableServo(0, 1).configureLogging(
                logTag + ":topWobbleGoalGrabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);

        arm = configuration.getHardwareMap().get(DcMotorEx.class, "arm");
        if (arm != null) {
            arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            // arm.setDirection(DcMotorEx.Direction.REVERSE);
        }
        configuration.register(this);
        // servoInit();
    }

    public void servoInit() {
        if (grabber!=null)
            grabber.setPosition(GRABBER_INIT);
        grabberIsClosed = true;
        armIsLow = false;
        armStop();
        // configuration.register(this);
    }

    public void armStop() {
        if (arm ==null) return;
        armToPos (arm.getCurrentPosition());
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setPower(0);
    }
    public Progress armToPos(int pos) {
      return armToPos(pos, false);
    }

    public Progress armToPos(int pos, boolean slow) {
        if (arm ==null) return null;
        arm.setTargetPosition(pos);
        if (pos > ARM_POS_DOWN - 100) {
            armIsLow = true;
        } else if (pos < ARM_POS_UP_AUTO + 100) {
            armIsLow = false;
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity((slow?ARM_SPEED_SLOW:ARM_SPEED));
        return new Progress() {
            public boolean isDone() {
                if (Math.abs(arm.getCurrentPosition() - arm.getTargetPositionTolerance()) < 5) {
                    return true;
                }
                return !arm.isBusy();
            }
        };
    }

    public void armInit() {
        armToPos(ARM_POS_INIT);
    }

    public void armUp() {
        armToPos(ARM_POS_UP);
        armIsLow = false;
    }

    public void armDown() {
        armToPos(ARM_POS_DOWN);
        armIsLow = true;
    }

    public void armAuto() {
        if (armIsLow)
            armUp();
        else
            armDown();
    }

    public void armUpInc(boolean forced) {
        if (arm ==null) return;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = arm.getCurrentPosition();
        if ((pos>= ARM_POS_MAX) && !forced) {
            //armStop();
            //return;
            pos = Math.min(pos + ARM_UNIT, ARM_POS_MAX);
        } else {
            pos = pos + ARM_UNIT;
        }
        armToPos(pos);
    }

    public void armDownInc(boolean forced) {
        if (arm ==null) return;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = arm.getCurrentPosition();
        if (pos<= ARM_POS_MIN && !forced) {
            //armStop();
            //return;
            pos = Math.max(pos-ARM_UNIT, ARM_POS_MIN);
        } else {
            pos = pos-ARM_UNIT;
        }
        armToPos(pos);
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
    public void releaseWobbleGoalCombo(boolean forAuto) {
        final String taskName = "release Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (forAuto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    grabberClose();
                    return armToPos(ARM_POS_DROP,true);
                }
            }, taskName);
            TaskManager.add(new Task() { // wait for 0.3 sec
                @Override
                public Progress start() {
                    runtime.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() {
                            return (runtime.seconds() >= 0.1);
                        }
                    };
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    grabberClose();
                    return armToPos(ARM_POS_DROP_HIGH,true);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPos(ARM_POS_UP);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_PARTIAL_CLOSE);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPos(ARM_POS_INIT);
            }}, taskName);
    }

    public void releaseWobbleGoalFastCombo() {
        final String taskName = "release Wobble Goal Fast Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPos(ARM_POS_DROP);
            }}, taskName);
        TaskManager.add(new Task() { // wait for 0.3 sec
            @Override
            public Progress start() {
                runtime.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (runtime.seconds() >= 0.1);
                    }
                };
            }
        }, taskName);
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
        armToPos(ARM_POS_INIT);
    }

    public void armUpCombo() { // put wobble goal to up position
        final String taskName = "arm Up Combo";
        if (!TaskManager.isComplete(taskName)) return;
        grabberPartialClose();
        armToPos(ARM_POS_UP);
    }
    public void armDownCombo() { // put wobble goal down position
        final String taskName = "arm Down Combo";
        if (!TaskManager.isComplete(taskName)) return;
        grabberOpen();
        armToPos(ARM_POS_DOWN);
    }
    public void armDownDownCombo() { // put wobble goal down position
        final String taskName = "arm Down Combo";
        if (!TaskManager.isComplete(taskName)) return;
        grabberOpen();
        armToPos(ARM_POS_DOWN_DOWN);
    }
    public void grabWobbleGoalCombo(boolean isHigh) {
        final String taskName = "grab Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        isGrabFromBottom = !isHigh;
        if (!armIsLow) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(GRABBER_OPEN);
                }}, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return armToPos(ARM_POS_DOWN);
                }}, taskName);
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    runtime.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() {
                            return (runtime.seconds() >= 0.1);
                        }
                    };
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armToPos(ARM_POS_DOWN_DOWN);
            }}, taskName);

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_CLOSE);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                runtime.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (runtime.seconds() >= 0.2);
                    }
                };
            }
        }, taskName);
        if (isHigh) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return armToPos(ARM_POS_UP);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return armToPos(ARM_POS_UP_AUTO);
                }
            }, taskName);
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
                return armToPos(ARM_POS_DOWN_DOWN);
            }}, taskName);
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
                return armToPos(ARM_POS_UP);
            }}, taskName);
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
            line.addData("arm", "%s", new Func<String>() {
                @Override
                public String value() {
                    String s = String.format("pos=%d, pw=%.1f, speed=%.1f", arm.getCurrentPosition(), arm.getPower(),
                            arm.getVelocity());
                    return s;
                }
            });
        }
    }

}



