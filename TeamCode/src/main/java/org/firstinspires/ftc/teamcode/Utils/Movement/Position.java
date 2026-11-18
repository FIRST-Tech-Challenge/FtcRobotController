package org.firstinspires.ftc.teamcode.Utils.Movement;

public class Position {

    private int type = 0;
    private Object[] extra;
    private long timeStamp = 0;
    private float[] xyr;
    public Position(float x,float y,float r) {
        xyr = new float[] {x,y,r};
    }

    public float getDistTo(Position pos) {
        return (float) Math.sqrt(Math.pow(pos.x()-xyr[0],2)+Math.pow(pos.y()-xyr[1],2));
    }

    public Position offset(float x, float y, float r) {
        return new Position(xyr[0]+x, xyr[1]+y,xyr[2]+r);
    }

    public long getTimeStamp() {
        return timeStamp;
    }
    public void setTimeStamp(long timeStamp) {
        this.timeStamp = timeStamp;
    }
    public Position invertPosi() {
        xyr[0] *= -1;
        xyr[1] *= -1;
        return this;
    }
    public Position invertedPosi() {
        return new Position(xyr[0]*-1,xyr[1]*-1,xyr[2]);
    }

    public Position sized(float mult) {
        Position pos = new Position(0,0,0);
        pos.setX(xyr[0]*mult);
        pos.setY(xyr[1]*mult);
        pos.setR(xyr[2]*mult);
        return pos;
    }

    public Position offset(Position p) {
        return new Position(xyr[0]+p.x(), xyr[1]+p.y(),xyr[2]+p.r());
    }

    public float x() {
        return xyr[0];
    }
    public float y() {
        return xyr[1];
    }
    public float r() {
        return xyr[2];
    }
    public Position setX(float x) {
        xyr[0] = x;
        return this;
    }
    public Position setY(float y) {
        xyr[1] = y;
        return this;
    }
    public Position setR(float r) {
        xyr[2] = r;
        return this;
    }

    /**
     * Rotates the position around origin by the given value in Degrees
     * @param rotation The angle (in Degrees) to rotate the movement
     */
    public Position rotateCoords(float rotation) {
        if(rotation != 0) {
            xyr[0] = (float) (-Math.sin(Math.toRadians(rotation))*xyr[1]+Math.cos(Math.toRadians(rotation))*xyr[0]);
            xyr[1] = (float) (Math.sin(Math.toRadians(rotation))*xyr[0]+Math.cos(Math.toRadians(rotation))*xyr[1]);
            xyr[2] += rotation;
        }
        return this;
    }

    @Override
    public String toString() {
        StringBuilder out = new StringBuilder("X: "+xyr[0]+",Y: "+xyr[1]+",R: "+xyr[2]);
        if(type == 1) {
            out.append(",Xv: "+extra[0]+",Yv: "+extra[1]);
        }
        return out.toString();
    }

    public Position round(int decimals) {
        xyr[0] = (float) (Math.round(xyr[0]*Math.pow(10,decimals))/Math.pow(10,decimals));
        xyr[1] = (float) (Math.round(xyr[1]*Math.pow(10,decimals))/Math.pow(10,decimals));
        xyr[2] = (float) (Math.round(xyr[2]*Math.pow(10,decimals))/Math.pow(10,decimals));
        return this;
    }

    /**
     * set the type of point/position
     * @param type 0 - Basic, generated points on the path.
     *             1 - Key point, points needed to generate the path.
     *             2 - Stop point, robot will stop here until OpMode re-starts it.
     *             3 - Pause, pause for a certain amount of time.
     * @param extraData Depends on type:
     *             1 - Vx (float),Vy (float).
     *             3 - Pause time in milliseconds (long).
     */
    public void setType(int type,Object[] extraData) {
        this.type = type;
        extra = extraData;
    }

    public int getType() {
        return type;
    }
    public Object[] getExtraData() {
        return extra;
    }
}
