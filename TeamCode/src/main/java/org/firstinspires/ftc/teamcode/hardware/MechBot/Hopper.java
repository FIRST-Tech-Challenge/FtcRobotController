package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.CRServo;
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

import static java.lang.Thread.sleep;

/**
 * FoundationHook spec:
 */
public class Hopper extends Logger<Hopper> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo feeder;
    private AdjustableServo holder;
    private AdjustableServo blocker;
    private CRServo ringLifter;
    /*private*/ public TouchSensor magLow;
    /*private*/ public TouchSensor magHigh;
    public DistanceSensor rangetouch;
    private ElapsedTime HopperTimer = new ElapsedTime();



    private final double FEEDER_IN = 0.45;
    private final double FEEDER_INIT = FEEDER_IN;
    private final double FEEDER_OUT = 0.92;

    private final double HOLDER_IN = 0.9;
    private final double HOLDER_INIT = HOLDER_IN;
    private final double HOLDER_OUT = 0.15;

    private final double BLOCKER_UP = 0.2;
    private final double BLOCKER_INIT = BLOCKER_UP;
    private final double BLOCKER_DOWN = 0.72;

    private boolean feederIsIn = true;
    private boolean holderIsIn = true;
    private boolean blockerIsUp = true;
    private boolean transferIsDown = true;
    private ElapsedTime runtime = new ElapsedTime();

    public boolean getTransferIsDown(){
        return transferIsDown;
    }
    @Override
    public String getUniqueName() {
        return "hopper";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public Hopper(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (ringLifter != null)
            servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        ringLifter = configuration.getHardwareMap().get(CRServo.class, "ringLifter");

        feeder = new AdjustableServo(0, 1).configureLogging(
                logTag + ":hopper", logLevel
        );
        feeder.configure(configuration.getHardwareMap(), "feeder");
        holder = new AdjustableServo(0, 1).configureLogging(
                logTag + ":hopper", logLevel
        );
        holder.configure(configuration.getHardwareMap(), "holder");
        blocker = new AdjustableServo(0, 1).configureLogging(
                logTag + ":hopper", logLevel
        );
        blocker.configure(configuration.getHardwareMap(), "blocker");
        configuration.register(feeder);
        configuration.register(holder);
        configuration.register(blocker);

        magLow = configuration.getHardwareMap().get(TouchSensor.class, "magLow");
        magHigh = configuration.getHardwareMap().get(TouchSensor.class, "magHigh");

        // servoInit();
      // configuration.register(this);
    }

    public void servoInit() {
        ringLifter.setPower(0);
        feeder.setPosition(FEEDER_INIT);
        feederIsIn = false;

        holder.setPosition(HOLDER_INIT);
        holderIsIn = true;
        blocker.setPosition(BLOCKER_INIT);
        blockerIsUp = true;
        //hookUp();
        // configuration.register(this);
    }

    public boolean touchingState() {
        return magLow.isPressed();
    }

    public double rangeReading() {
        return rangetouch.getDistance(DistanceUnit.CM);
    }

    public void feederIn() {
        feeder.setPosition(FEEDER_IN);
        feederIsIn = true;
    }

    public void feederOut() {
        feeder.setPosition(FEEDER_OUT);
        feederIsIn = false;
    }

    public void feederAuto() throws InterruptedException {
        feederOut();
        sleep(250);
        feederIn();
    }

    public void holderIn() {
        holder.setPosition(HOLDER_IN);
        holderIsIn = true;
    }

    public void holderOut() {
        holder.setPosition(HOLDER_OUT);
        holderIsIn = false;
    }

    public void holderAuto() throws InterruptedException {
        if(holderIsIn)
            holderOut();
        else
            holderIn();
    }

    public void blockerUp() {
        blocker.setPosition(BLOCKER_UP);
        blockerIsUp = true;
    }

    public void blockerDown() {
        blocker.setPosition(BLOCKER_DOWN);
        blockerIsUp = false;
    }

    public void blockerAuto() throws InterruptedException {
        if(blockerIsUp)
            blockerDown();
        else
            blockerUp();
    }

    public void transferUp(){
        if (ringLifter==null) return;
        ringLifter.setPower(-1);
    }

    public void transferDown(){
        if (ringLifter==null) return;
        ringLifter.setPower(1);
    }

    public void transferStop(){
        if (ringLifter==null) return;
        ringLifter.setPower(0);
    }


    public void transferUpAuto() throws InterruptedException {
        if (ringLifter == null) return;
        if (!transferIsDown) return;
        ringLifter.setPower(-1);
        sleep(1500);
        ringLifter.setPower(-0.85);
        sleep(100);
        ringLifter.setPower(0.6);
        sleep(50);
        ringLifter.setPower(0);
        transferIsDown = false;
    }

    public void transferUpCombo(boolean forAuto) throws InterruptedException {
        if (ringLifter == null) return;
        final String taskName = "Transfer Up Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                blockerUp();
                ringLifter.setPower(-1);
                HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (HopperTimer.seconds()>=1.5 || magHigh.isPressed());
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                holderOut();
                HopperTimer.reset();
                ringLifter.setPower(-0.85);
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (HopperTimer.milliseconds()>=300);
                    }
                }; }}, taskName);
        if (!forAuto) {
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                holderOut();
                HopperTimer.reset();
                ringLifter.setPower(0.5);
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (HopperTimer.milliseconds()>=10);
                    }
                }; }}, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(0);
                transferIsDown = false;
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (true);
                    }
                }; }}, taskName);
    }

    public void transferDownAuto() throws InterruptedException {
        if (ringLifter == null) return;
        // if (transferIsDown) return;
        double iniTime = System.currentTimeMillis();
        ringLifter.setPower(1);
        while(!magLow.isPressed() && (System.currentTimeMillis() - iniTime < 2500) ) {
            sleep(5);
        }
        ringLifter.setPower(-0.5);
        sleep(50);
        ringLifter.setPower(0);
        transferIsDown = true;
    }

    public void hopperDownCombo() throws InterruptedException {
        if (ringLifter == null) return;
        holderIn();
        ringLifter.setPower(1);
        transferDownCombo();
        TaskManager.processTasks();
    }

    public void hopperUpCombo(boolean forAuto) throws InterruptedException {
        if (ringLifter == null) return;
        // if (!transferIsDown) return;
        ringLifter.setPower(-1);
        transferUpCombo(forAuto);
        TaskManager.processTasks();
    }

    public void transferDownCombo() throws InterruptedException {
        if (ringLifter == null) return;
        final String taskName = "Transfer Down Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                holderIn();
                HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (HopperTimer.seconds()>0.2);
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(1);
                HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (magLow.isPressed() || (HopperTimer.seconds()>=2.5));
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(-0.5);
                HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (HopperTimer.milliseconds()>60);
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(0);
                transferIsDown = true;
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (true);
                    }
                }; }}, taskName);
    }
    public void transferShakeCombo() throws InterruptedException {
        if (ringLifter == null) return;
        final String taskName = "Transfer Shake Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(-1);
                HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (HopperTimer.seconds()>=0.2);
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(1);
                HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (magLow.isPressed() || (HopperTimer.seconds()>=2));
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(-0.1);
                HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (HopperTimer.milliseconds()>=50);
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                ringLifter.setPower(0);
                transferIsDown = true;
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (true);
                    }
                }; }}, taskName);
    }


    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (ringLifter != null) {
            line.addData("Transfer", "pow=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return ringLifter.getPower();
                }
            });
        }

        if (feeder != null) {
            line.addData("Feeder", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return feeder.getPosition();
                }
            });
        }

        if (holder != null) {
            line.addData("Holder", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return holder.getPosition();
                }
            });
        }

        if (blocker != null) {
            line.addData("Blocker", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return blocker.getPosition();
                }
            });
        }

        if (magLow != null) {
            line.addData("TouchLow", "pressed=%s", new Func<String>() {
                @Override
                public String value() {
                    return (magLow.isPressed()?"Yes":"No");
                }
            });
        }

        if (magHigh != null) {
            line.addData("TouchHigh", "pressed=%s", new Func<String>() {
                @Override
                public String value() {
                    return (magHigh.isPressed()?"Yes":"No");
                }
            });
        }


    }

}



