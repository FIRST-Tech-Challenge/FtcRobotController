package org.firstinspires.ftc.teamcode.utility;

public class pose {
    public double x, y, r;// radians

    public pose(double x, double y, double r){
        this.x = x;
        this.y = y;
        this.r = r;
    }

    /**
     * The added vector is rotated about its origin by the rotation of the parent
     */
    public void translateRelative(pose p){
        point rotated = point.rotate(p.x, p.y, this.r);
        this.x += rotated.x;
        this.y += rotated.y;
        this.r += p.r;
    }

    public String toString(){
        return String.format("pose %.3f %.3f %.3f", x, y, r);
    }
}
