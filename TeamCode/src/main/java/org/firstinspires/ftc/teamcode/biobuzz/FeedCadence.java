package org.firstinspires.ftc.teamcode.biobuzz;

/** Non-blocking indexer pulse and mandatory OFF gap. Not a ball/shot counter.
 * Call on every loop, including zero demand. Reverse unjamming remains immediate. */
public final class FeedCadence {
    public static final double PULSE_SECONDS=.18, GAP_SECONDS=.5;
    private final double pulseSeconds,gapSeconds;
    private double until=Double.NEGATIVE_INFINITY,readyAt=Double.NEGATIVE_INFINITY,last=Double.NEGATIVE_INFINITY;
    private boolean feeding;
    public FeedCadence(){this(PULSE_SECONDS,GAP_SECONDS);}
    public FeedCadence(double pulseSeconds,double gapSeconds){
        if(!Double.isFinite(pulseSeconds)||pulseSeconds<=0||!Double.isFinite(gapSeconds)||gapSeconds<.5)
            throw new IllegalArgumentException("Invalid indexer timing");
        this.pulseSeconds=pulseSeconds;this.gapSeconds=gapSeconds;
    }
    public double apply(double demand,double now){
        if(!Double.isFinite(now)||now<last||!Double.isFinite(demand)){feeding=false;readyAt=Double.POSITIVE_INFINITY;return 0;}
        last=now;
        if(feeding&&(demand<=0||now>=until)){feeding=false;readyAt=now+gapSeconds;}
        if(demand<0){readyAt=now+gapSeconds;return Math.max(-1,demand);}
        if(demand==0)return 0;
        if(!feeding&&now>=readyAt){feeding=true;until=now+pulseSeconds;}
        return feeding?Math.min(1,demand):0;
    }
}
