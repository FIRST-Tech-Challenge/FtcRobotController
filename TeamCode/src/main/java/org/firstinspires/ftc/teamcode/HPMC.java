package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class HPMC {
    static final float NANOSECONDS_PER_SECOND = 1000000000;
    static final double LOOK_AHEAD_TIME = 0.075;
    static final double FINE_POWER_SCALE =  0.0002;
    static final double MAX_POWER_CHANGE = 2.0;
    static final long MS_PER_NS = 1000000;


    int historySize = 3;
    static long tickMillis = 50; //in milliseconds
    double tickSeconds = tickMillis / 1000.0;
    //For Smart Ticks
    private long nextWake = 0;
    public int currentPosition = 0;
    private double desiredVelocity = 0;
    double currentVelocity = 0;
    double power = 0;
    double accelleration = 0;
    float updatesPerSecond = 0;
    double velocitySoon = 0;
    double change = 0;
    double maxSpeed = 0;

    long lastUpdateTime = 0;
    DcMotorEx motor = null;

    //The name of the motor for debugging;
    String label = null;



    public enum MoveState { ACCELERATING, AT_SPEED, STOPPING, DONE, BRAKING, STUPID};
    public enum Direction {FORWARD, REVERSE};

    //smooth move variables
    MoveState smState = MoveState.DONE;
    long smAccelerationTick;
    long smStartPosition;
    long smEndPosition;
    double smStartSpeed;
    double smVelocity;
    long  smDistance;
    long smAccelerationTicks;
    //long smDesiredProgress;
    long smStartStopping;
    long smDesiredPosition;
    long smDecelerationTicks;
    long smDecelerationTick;
    boolean smEndStopped = true;
    boolean braking = false;





    ArrayList<Integer> positionList = new ArrayList<Integer>();
    ArrayList<Long> timeList = new ArrayList<Long>();

    public HPMC(DcMotorEx setMotor) {
        motor = setMotor;
    }

    public HPMC(HardwareMap hardwareMap, String motorString, double maxSpeedIn) {
        motor = hardwareMap.get(DcMotorEx.class, motorString);
        maxSpeed = maxSpeedIn;
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
        updateCurrentVelocity();
    }

    public HPMC(DcMotorEx setMotor, double maxSpeedIn) {
        motor = setMotor;
        maxSpeed = maxSpeedIn;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    //takes a power, calculates a desired velocity and autoAdjusts to it
    public void setPowerAuto(double powerFactor) {
        if (powerFactor > 1) powerFactor = 1;
        if (powerFactor < -1) powerFactor = -1;
        desiredVelocity = maxSpeed * powerFactor;
        updateCurrentVelocity();
        autoAdjust();
    }

    public void setPowerManual(double powerIn) {
        power = powerIn;
        motor.setPower(power);
    }

    public void setDesiredVelocity(double newDesiredVelocity) {
        desiredVelocity = newDesiredVelocity;
        updateCurrentVelocity();
        autoAdjust();
    }

    public void updateCurrentVelocity() {
        long nanotime = System.nanoTime();
        //Don't update if it's less than 10 ms since the last update
        if ( (nanotime - lastUpdateTime) <   (10 * MS_PER_NS) ) {
            debug("Skipping double update");
            return;
        } else if (nanotime - lastUpdateTime > (100* MS_PER_NS) ) {
            debug("currentPosition and time lists are outdated.  Clearing: " + (nanotime-lastUpdateTime)/1000000.0);
            positionList.clear();
            timeList.clear();
        }
        currentPosition = motor.getCurrentPosition();
        positionList.add(currentPosition);
        timeList.add(nanotime);
        lastUpdateTime = nanotime;
        if (positionList.size() > historySize) {
            positionList.remove(0);
        }
        if (timeList.size() > historySize) {
            timeList.remove(0);
        }
        //set currentVelocity to
        if (timeList.size() > 2) {
            int start = 0;
            int end = timeList.size() - 1;
            int middle = end / 2;

            long elapsed = timeList.get(end) - timeList.get(start);
            int distance = positionList.get(end) - positionList.get(start);
            long oldDistance = positionList.get(middle) - positionList.get(start);
            long newDistance = positionList.get(end) - positionList.get(middle);
            long oldElapsed = timeList.get(middle) - timeList.get(start);
            long newElapsed = timeList.get(end) - timeList.get(middle);
            if ((elapsed > 0) && (oldElapsed > 0) && (newElapsed > 0)) {
                currentVelocity = distance * (NANOSECONDS_PER_SECOND / elapsed);
                updatesPerSecond = (timeList.size() * (NANOSECONDS_PER_SECOND / elapsed));
                double oldSpeed = oldDistance * (NANOSECONDS_PER_SECOND / oldElapsed);
                double newSpeed = newDistance * (NANOSECONDS_PER_SECOND / newElapsed);
                double secondsElapsed = (newElapsed / NANOSECONDS_PER_SECOND);

                accelleration = (newSpeed - oldSpeed) / secondsElapsed;
            } else {
                debug("Not enough history.  Using zeros");
                currentVelocity = 0;
                accelleration = 0;
            }
        } else {
            currentVelocity = 0;
            accelleration=0;
        }
    }


    public void manualAdjust(double velocity) {
        power = velocity / maxSpeed;
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        motor.setPower(power);
    }


    //Set motor power based on desiredVelocity

    public void autoAdjust() {

        velocitySoon = currentVelocity + (accelleration * LOOK_AHEAD_TIME);
        /*if (braking) {
            //reset power to a sane value so if we no longer need to brake, the adjustments have a reasonable starting point
            power = currentVelocity / maxSpeed;
            braking = false;
        }*/
        double difference =  desiredVelocity - velocitySoon;
        if ( Math.abs(desiredVelocity) < 1) {
            //we want to be stopped.  Zero the power.
            change = -power;
            //System.out.println(String.format("Change from zeroing: %.4f", change));
        //} else if ( (velocitySoon / desiredVelocity) > 1.2 && Math.abs(difference) > 400 ) {
        //    power = recoverLostSign(0.15, smVelocity);
        //    braking = true;
            //System.out.println(String.format("Change from overspeed: %.4f", change));
        } else if ( (velocitySoon / desiredVelocity) > 1.01) {
            change =  (power * (desiredVelocity / velocitySoon)) - power;
            //System.out.println(String.format("Change from overspeed: %.4f", change));
        } else {
            change = FINE_POWER_SCALE * difference;
            //System.out.println(String.format("Change from power scale: %.4f", change));
        }

        /*
        if (Math.abs(change) > MAX_POWER_CHANGE) {
            if (change < 0) {
                change = -MAX_POWER_CHANGE;
            } else {
                change = MAX_POWER_CHANGE;
            }
        }*/

        power = power + change;
        //debug(String.format("Power: %.4f change: %.4f velocitySoon: %.2f  desiredVelocity: %.2f diff: %.2f pos: %d", power, change, velocitySoon, desiredVelocity, difference,currentPosition));
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        double motorPower = 0;
        if (Math.abs(desiredVelocity)  > 10 || ( Math.abs(power) > 0.003 ) ) {
            motorPower = power * 0.8 + recoverLostSign(0.15f, power);
        }
        debug(String.format("Power: %.3f MotorPower: %.3f", power, motorPower));
        motor.setPower(motorPower);
    }


    public void smoothMoveSetup(double distance, double power, double accelerationTicks, double decelerationTicks, Direction direction, boolean endStopped) {
        updateCurrentVelocity();
        smStartPosition = currentPosition;

        if (distance < 0) {
            //reverse direction
            switch (direction) {
                case FORWARD:
                    direction = Direction.REVERSE;
                    break;
                case REVERSE:
                    direction = Direction.FORWARD;
            }
        }

        smDistance = (long) Math.abs(distance);

        if (direction == Direction.FORWARD) {
            smEndPosition = smStartPosition + smDistance;
        } else {
            smEndPosition = smStartPosition - smDistance;
        }

        desiredVelocity = currentVelocity;
        smStartSpeed = currentVelocity;
        smAccelerationTicks = (long) accelerationTicks;
        smAccelerationTick = 0;
        //smDesiredProgress = 0;
        smDesiredPosition = currentPosition;
        smDecelerationTicks = (long) decelerationTicks;
        smDecelerationTick = 0;
        smEndStopped = endStopped;

        if (smAccelerationTicks<1) { smAccelerationTicks = 1;}
        if (direction == Direction.FORWARD) {
            smVelocity = power*maxSpeed;
        } else {
            smVelocity = -power*maxSpeed;
        }

        smState = MoveState.ACCELERATING;
        double  distanceDecelerating = (smVelocity / 2 ) * ((accelerationTicks+1) * tickMillis / 1000.0);
        if (endStopped) {
            smStartStopping = (long) (smDistance - Math.abs(distanceDecelerating));
            if (smStartStopping < (smDistance *0.4) ) {
                smStartStopping = (long) (smDistance *0.4);
            }
        } else {
            smStartStopping = smDistance;
        }
        debug2(String.format("Setting Up SM:  Distance: %d Power: %.2f  AT: %d D: %s SD %.2f   Start Slowing At:  %d", smDistance, power, smAccelerationTicks, direction.toString(), distanceDecelerating, smStartStopping));

    }

    public double percentComplete() {
        return ( moved() / (double) smDistance);
    }


    public String getMoveState() {
        String percent;
        if (smDistance > 0) {
            percent = String.format(" %.1f%% %d",  (double) moved()*100 / (double) smDistance, smDistance - moved() );

            percent = String.format(" %.1f%% %d",  (double) moved()*100 / (double) smDistance, distanceLeft() );

        } else {
            percent = "---";
        }
        if (smState == MoveState.ACCELERATING) {
            return  percent + "-A";
        } else if (smState == MoveState.AT_SPEED) {
            return  percent + "-R";
        } else if ( smState == MoveState.BRAKING) {
            return  percent + "-B";
        } else if ( smState == MoveState.STOPPING) {
            return  percent + "-S" + (smDecelerationTicks - smDecelerationTick);
        } else if ( smState == MoveState.DONE) {
            return  percent + "-D";
        } else {
            return percent + "-?";
        }
    }

    public boolean smTick() {
        return smTick(0);
    }

    public boolean smTick(double targetPercentDone) {
        double catchupVelocity = 0;
        //If we will reach the end of our travel before we tick again, stop now.
        if ((smState != MoveState.DONE) && (distanceLeft()  <  distancePerTick(Math.abs(currentVelocity)/ 2) ) || ( distanceLeft() < 3) ) {
            debug2(String.format("DONE  Moved: %d of %d", moved(), smDistance));
            stopIfEndingStopped();
            smState = MoveState.DONE;
            return(false);
        }

        if (targetPercentDone > 0) {
            double targetPosition = (smEndPosition - smStartPosition) * targetPercentDone + smStartPosition;
            double catchupDistance = targetPosition - currentPosition;
            //try to catch up 1/2 of the remaining distance this tick;
            catchupVelocity = catchupDistance / (tickSeconds * 2);
            debug(String.format("Catchup Data:  TPD: %.4f  targetPosition: %.1f  currentPosition: %d  catchupDistance: %.4f, catchupVelocity: %.1f", targetPercentDone, targetPosition, currentPosition ,catchupDistance, catchupVelocity));
            //catchupVelocity = 0;   //Yea.  I had bugs.
        }
        //debug(String.format("Tick starting.  Moved %d of %d.  State: %s", moved(), smDistance, smState.toString() ));
        switch (smState) {
            case ACCELERATING:
                smAccelerationTick++;
                double targetPercentSpeed = (smAccelerationTick) / (double) smAccelerationTicks;
                double currentPercentVelocity = currentVelocity / smVelocity;
                if (smAccelerationTick == 1) {
                    //set power to a sane starting point for acceleration regardless of what autoAdjust had set it to previously
                    power = smVelocity / maxSpeed * 0.2;
                } else if ((moved() > (smDistance / 3.0)) || ((currentVelocity / smVelocity) > 0.9)) {
                    //End Accelerating if we're more than 1/3rd of the way there, or if we're up to speed.
                    debug(String.format("Switching to AT_SPEED  (%d > %d / 3) or ( %.1f / %.1f) > 0.9", moved(), smDistance, currentVelocity, smVelocity));
                    smState = MoveState.AT_SPEED;
                }

                double estimatedPercentSpeed = (smAccelerationTick - 1) / (double) smAccelerationTicks;
                //This is what we want to move, but we target faster because acceleration takes time.

                //smDesiredProgress += movedPerTick(estimatedPercentSpeed * smVelocity);
                targetPercentSpeed = (smAccelerationTick + 1) / (double) smAccelerationTicks;
                targetPercentSpeed = (targetPercentSpeed * 0.8) + 0.2;  //start at 20% speed and ramp up from there
                desiredVelocity = ((smVelocity - smStartSpeed) * (targetPercentSpeed)) + smStartSpeed + catchupVelocity;

                autoAdjust();
                debug(String.format("ACC Spd: %.1f%% (%.1f of %.1f) Moved: %d  TPD:%.1f CatchupV: %.1f  Pow:%.1f", currentPercentVelocity * 100.0, currentVelocity, desiredVelocity, moved(), targetPercentDone, catchupVelocity, power));


                if (moved() > smStartStopping) {
                    if (smEndStopped) {
                        smState = MoveState.STOPPING;
                    } else {
                        smState = MoveState.DONE;
                        return (false);
                    }
                } else if (smAccelerationTick >= smAccelerationTicks) {
                    smState = MoveState.AT_SPEED;
                }

                //debug(String.format("ACC %.2f%% DS: %.2f  CS: %.2f Moved: %d(%d) of %d PWR:%.2f", targetPercentSpeed, desiredVelocity, currentVelocity, moved, smDesiredProgress, smStartStopping, power));
                return (true);
            case AT_SPEED:
                smDesiredPosition += distancePerTick(smVelocity);
                //smDesiredProgress += movedPerTick(smVelocity);

                //Increase speed by 10% if this wheel is behind;
                //long ahead = smDesiredProgress - moved();

                desiredVelocity = smVelocity + catchupVelocity;
                autoAdjust();


                if (moved() > smStartStopping ) {
                    if (smEndStopped) {
                        smState = MoveState.STOPPING;
                    } else {
                        smState = MoveState.DONE;
                        return(false);
                    }
                }
                debug(String.format("A_S DS:%.2f  CS:%.2f Moved:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, moved(), smStartStopping, power));
                return(true);
            case STOPPING: {

                long stoppingDistanceLeft = (smDistance - moved());
                smDecelerationTick++;
                long ticksLeft = smDecelerationTicks - smDecelerationTick - 1;
                if (ticksLeft < 1 ) {
                    stopIfEndingStopped();
                    smState = MoveState.DONE;
                    return (false);
                }
                //double ratio = (0.7 * smDecelerationTick  / (double) smDecelerationTicks) + 0.1;
                //desiredVelocity = (smSpeed  - (smSpeed * ratio));

                double timeLeft = (ticksLeft  * tickSeconds);
                desiredVelocity = (stoppingDistanceLeft / timeLeft * 2.2);
                desiredVelocity = recoverLostSign(desiredVelocity, smVelocity);
                desiredVelocity = desiredVelocity + catchupVelocity;

                //estimated braking speed change is 500/50ms;
                double minStoppingTicks = Math.abs(currentVelocity) / 600;
                long minStoppingDistance = (long) ( Math.abs(currentVelocity) * minStoppingTicks * tickSeconds)/2;

                if (stoppingDistanceLeft > minStoppingDistance) {
                    autoAdjust();
                    debug(String.format("SLO DSPD: %.2f  SPD: %.2f Left: %d MSD:%d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, minStoppingDistance,  ticksLeft, moved(), smDistance, power));
                    return (true);
                } else {
                    stopIfEndingStopped();
                    debug(String.format("BRK DSPD: %.2f  SPD: %.2f Left: %d MSD:%d TL:%d M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity, stoppingDistanceLeft, minStoppingDistance, ticksLeft, moved(), smDistance, power));
                    return (true);
                }
            }
            case DONE:
                //debug(String.format("DONE DSPD: %.2f  SPD: %.2f M:%d of %d PWR:%.2f", desiredVelocity, currentVelocity,  moved(), smDistance, power));
                stopIfEndingStopped();
                return(false);
        }
        return false;
    }

    double recoverLostSign(double number, double source) {
        if (source<0) {
            return(-number);
        }
        return number;
    }

    void stopIfEndingStopped() {
        if (smEndStopped) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            power = 0;
            motor.setPower(0);
            desiredVelocity = 0;
        }
    }

    public void setTickTime(long tickTimeIn) {
        tickMillis = tickTimeIn;
        double tickSeconds = tickMillis / 1000.0;
    }
    public void setLabel(String string) { label = string;}
    public void setHistorySize(int size) { historySize = size;}
    public double getPower() { return power; }
    public double getCurrentVelocity() { return currentVelocity; }
    public double getDesiredVelocity() { return desiredVelocity;}
    public double getAccelleration()  { return accelleration; }
    public double getVelocitySoon()  { return velocitySoon; }
    public double getUpdatesPerSecond() { return updatesPerSecond;}
    public int getCurrentPosition() { return currentPosition; }

    public String getSMStatus() {
        return String.format("State: %s  Moved: %d Speed: %.2f  Desired: %.2f Power: %.2f StoppingAt: %d",
                smState.toString(),
                Math.abs(currentPosition - smStartPosition),
                currentVelocity, desiredVelocity, power, smStartStopping);
    }

    void debug(String string) {
        if (label != null) {
            System.out.println("M: " + label + " " + string);
        }
    }
    void debug2(String string) {
        if (label != null) {
            System.out.println("M: " + label + " " + string);
        }
    }


    long moved() {
        if (smEndPosition < smStartPosition) {
            return(smStartPosition - currentPosition);
        } else {
            return(currentPosition - smStartPosition);

        }
    }

    long distanceLeft() {
        return smDistance - moved();
    }

    public int smGetMovementError() {
        return (int) Math.abs(distanceLeft());
    }

    long distancePerTick(double velocity) {
        return (long) (velocity * tickSeconds);
    }
    long movedPerTick(double velocity) {
        return Math.abs(distancePerTick(velocity));
    }
}




