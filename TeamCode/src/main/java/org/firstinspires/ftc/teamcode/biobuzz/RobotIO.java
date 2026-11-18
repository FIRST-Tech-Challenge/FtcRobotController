package org.firstinspires.ftc.teamcode.biobuzz;

/** The only boundary between autonomous decisions and hardware/simulation. */
public interface RobotIO {
    /** FTC overrides this with isStopRequested; desktop interruption also cancels work. */
    default boolean stopRequested() { return Thread.currentThread().isInterrupted(); }
    double nowSeconds();
    Sensors readSensors();
    void apply(Outputs outputs);

    /** Inches, field X/Y, radians counterclockwise from +X. */
    final class Pose {
        public final double x, y, heading;
        public Pose(double x, double y, double heading) { this.x=x; this.y=y; this.heading=heading; }
        public double distance(Pose other) { return Math.hypot(x-other.x,y-other.y); }
        public static double angle(double a) { return Math.atan2(Math.sin(a),Math.cos(a)); }
        public boolean finite() { return Double.isFinite(x)&&Double.isFinite(y)&&Double.isFinite(heading); }
        /** Reject corrupt readings before any SDK angle normalization or route search. */
        public static Pose validatedOdometry(double x,double y,double rawHeading) {
            if(!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(rawHeading)
                    ||Math.abs(x)>144||Math.abs(y)>144||Math.abs(rawHeading)>1e6)return null;
            return new Pose(x,y,angle(rawHeading));
        }
    }
    final class Target {
        public final int id;
        /** Limelight convention: tx is degrees, right/clockwise positive. */
        public final double tx, ty;
        /** Measured tag height above the tiles, inches; NaN without calibrated 3D pose. */
        public final double heightInches;
        /** Color-pipeline measurement; no world position or source/provenance is exposed. */
        public final double areaPercent;
        public final String color;
        public Target(int id,double tx,double ty) { this(id,tx,ty,Double.NaN); }
        public Target(int id,double tx,double ty,double heightInches) {
            this(id,tx,ty,heightInches,Double.NaN,"");
        }
        public Target(int id,double tx,double ty,double heightInches,double areaPercent,String color) {
            this.id=id; this.tx=tx; this.ty=ty; this.heightInches=heightInches;
            this.areaPercent=areaPercent;this.color=color==null?"":color;
        }
    }
    final class Vision {
        public final boolean valid;
        public final int pipeline;
        public final double capturedAt;
        public final Target[] targets;
        public Vision(boolean valid,int pipeline,double capturedAt,Target[] targets) {
            this.valid=valid; this.pipeline=pipeline; this.capturedAt=capturedAt; this.targets=targets.clone();
        }
        public boolean fresh(double now,int expectedPipeline) {
            // A Limelight frame can reach the Driver Hub late while the camera
            // is switching pipelines.  One second is still short enough to
            // discard a previous-location observation during this fixed shot.
            return valid&&pipeline==expectedPipeline&&now-capturedAt>=0&&now-capturedAt<=1.0;
        }
        public static Vision empty(int pipeline,double now) { return new Vision(false,pipeline,now,new Target[0]); }
    }
    final class Sensors {
        public final Pose pose;
        public final double sampledAt;
        public final boolean odometryValid, cameraConnected;
        /** -1 means unavailable; never manufacture a full/empty count. */
        public final int ballCount;
        /** Type-verified occupied pollen slots, -1 if unavailable; never inferred from time. */
        public final int pollenCount;
        public final Vision vision;
        public Sensors(Pose pose,double sampledAt,boolean odometryValid,boolean cameraConnected,int ballCount,Vision vision) {
            this(pose,sampledAt,odometryValid,cameraConnected,ballCount,-1,vision);
        }
        public Sensors(Pose pose,double sampledAt,boolean odometryValid,boolean cameraConnected,int ballCount,int pollenCount,Vision vision) {
            this.pose=pose; this.sampledAt=sampledAt; this.odometryValid=odometryValid;
            this.cameraConnected=cameraConnected; this.ballCount=ballCount;this.pollenCount=pollenCount; this.vision=vision;
        }
    }
    final class Outputs {
        /** Robot relative forward, right strafe, clockwise turn; normalized powers. */
        public double forward, strafeRight, turnClockwise;
        public double shooterPower, feederPower, intakePower;
        /** 0 = hive AprilTags / camera up, 1 = balls / camera down. */
        public int pipeline;
        /** Front-left, front-right, back-left, back-right, after mecanum normalization. */
        public double[] wheelPowers() { return wheelPowers(1); }
        /** Hardware-only longitudinal calibration. Never invert strafe/turn or field poses. */
        public double[] wheelPowers(double forwardPowerSign) {
            if(forwardPowerSign!=1&&forwardPowerSign!=-1)throw new IllegalArgumentException("Forward power sign must be +1 or -1");
            double y=forward*forwardPowerSign;
            double fl=y+strafeRight+turnClockwise,fr=y-strafeRight-turnClockwise,
                bl=y-strafeRight+turnClockwise,br=y+strafeRight-turnClockwise;
            double scale=Math.max(1,Math.max(Math.max(Math.abs(fl),Math.abs(fr)),Math.max(Math.abs(bl),Math.abs(br))));
            return new double[]{fl/scale,fr/scale,bl/scale,br/scale};
        }
        public Outputs copy() {
            Outputs o=new Outputs(); o.forward=forward; o.strafeRight=strafeRight; o.turnClockwise=turnClockwise;
            o.shooterPower=shooterPower; o.feederPower=feederPower; o.intakePower=intakePower; o.pipeline=pipeline; return o;
        }
    }
}
