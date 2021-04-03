package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * FoundationHook spec:
 */
public class BottomWobbleGoalGrabber extends Logger<BottomWobbleGoalGrabber> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo pivot;
    private AdjustableServo grabber;

    private final double PIVOT_UP = 0.3;
    private final double PIVOT_INIT = 0.29;
    private final double PIVOT_DOWN = 0.66;

    private final double GRABBER_OPEN = 0.5;
    //private final double GRABBER_INIT = 0.78;
    //private final double GRABBER_CLOSE = 0.78;
    private final double GRABBER_CLOSE = 0.80;
    private final double GRABBER_INIT = GRABBER_CLOSE;

    private boolean pivotIsDown = false;
    private boolean grabberIsClosed = true;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "bottomWobbleGoalGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public BottomWobbleGoalGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (grabber != null)
            servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        grabber = new AdjustableServo(0, 1).configureLogging(
                logTag + ":bottomWobbleGoalGrabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "b_grabber");
        configuration.register(grabber);

        pivot = new AdjustableServo(0, 1).configureLogging(
                logTag + ":bottomWobbleGoalGrabber", logLevel
        );
        pivot.configure(configuration.getHardwareMap(), "pivot");
        configuration.register(pivot);
        configuration.register(this);
        // servoInit();
    }

    public void servoInit() {
        grabber.setPosition(GRABBER_CLOSE);
        pivot.setPosition(PIVOT_INIT);
        pivotIsDown = false;
        grabberIsClosed = true;
    }

    public void pivotUpInc() {
        double pos = Math.min(pivot.getPosition()+0.01,0.999);
        pivot.setPosition(pos);
        if (pos>=PIVOT_UP)
            pivotIsDown = false;
    }
    public void pivotUpDec() {
        double pos = Math.max(pivot.getPosition()-0.01,0.001);
        pivot.setPosition(pos);
        if (pos<=PIVOT_DOWN)
            pivotIsDown = true;
    }

    public void pivotUp() {
        pivot.setPosition(PIVOT_UP);
        pivotIsDown = false;
    }

    public void pivotDown() {
        pivot.setPosition(PIVOT_DOWN);
        pivotIsDown = true;
    }

    public void pivotAuto() {
        if (pivotIsDown) {
            pivotUp();
        } else {
            pivotDown();
        }
    }

    private Progress movePivot(double position) {
        double adjustment = Math.abs(position - pivot.getPosition());
        pivot.setPosition(position);
        if (position>= PIVOT_DOWN -0.1)
            pivotIsDown=true;
        else {
            pivotIsDown = false;
        }
        // 600 ms per 180 degree
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 900);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void grabberOpen(){
        grabber.setPosition(GRABBER_OPEN);
        grabberIsClosed = false;
    }

    public void grabberClose(){
        grabber.setPosition(GRABBER_CLOSE);
        grabberIsClosed = true;
    }

    public void grabberAuto(){
        if (grabberIsClosed) {
            grabberOpen();
        } else {
            grabberClose();
        }
    }

    public void releaseWobbleGoalCombo(boolean pivotIsUp) {

        final String taskName = "release Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return movePivot(PIVOT_DOWN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }
        }, taskName);
        if (!pivotIsUp)
            return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return movePivot(PIVOT_UP);
            }
        }, taskName);
    }

    public void grabWobbleGoalCombo() {
        final String taskName = "grab Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return movePivot(PIVOT_DOWN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_CLOSE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return movePivot(PIVOT_UP);
            }
        }, taskName);
    }

    private Progress moveGrabber(double position) {
        double adjustment = Math.abs(position - grabber.getPosition());
        grabber.setPosition(position);
        //if (position<= GRABBER_CLOSE + 0.1)
        if (position<= GRABBER_CLOSE + 0.02)
            grabberIsClosed=true;
        else {
            grabberIsClosed = false;
        }
        // 600 ms per 180 degree
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 900);
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
            line.addData("B-Grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }

        if (pivot != null) {
            line.addData("Pivot", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return pivot.getPosition();
                }
            });
        }
    }

}



