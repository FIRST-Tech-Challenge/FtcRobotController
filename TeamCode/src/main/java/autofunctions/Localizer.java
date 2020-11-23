package autofunctions;

public class Localizer {

    public double r1 = 0;
    public double l1 = 0;
    public double r2 = 0;
    public double l2 = 0;
    public double theta = 0;

    public double r1fn = 0;
    public double l1fn = 0;
    public double r2fn = 0;
    public double l2fn = 0;

    public double dx = 0;
    public double dy = 0;

    public int counter = 0;

    public final double robotWidth = 28;
    public final double robotLength = 33;
    public final double robotRadius = Math.sqrt(Math.pow(robotWidth/2, 2) + Math.pow(robotLength/2, 2));
    public final double centerTheta = Math.atan2(robotLength,robotWidth);


    public void update(double r1, double l1, double r2, double l2, double heading){
        theta = heading;
//        this.r1 = r1;
//        this.r2 = r2;
//        this.l1 = l1;
//        this.l2 = l2;

        r1fn += r1/3;
        l1fn += l1/3;
        r2fn += r2/3;
        l2fn += l2/3;

        counter++;
        if(counter == 3){
            counter = 0;
            this.r1 = r1fn;
            this.r2 = r2fn;
            this.l1 = l1fn;
            this.l2 = l2fn;
            r1fn = 0;
            l1fn = 0;
            r2fn = 0;
            l2fn = 0;
        }

    }

    public double getAngle(){
        double cd = l1-r1;
        double out = Math.atan2(cd, robotWidth);
        return Math.toDegrees(out);
    }

    public double getX(){
        double a = robotRadius;
        double n9 = Math.toRadians(90);
        double d = Math.sqrt(l2*l2+a*a-(2*Math.abs(l2)*a*Math.cos(centerTheta+n9)));
        dx = d;
        double phi = Math.asin((a*Math.sin(centerTheta+n9))/d);
        double x = d*Math.cos(Math.toRadians(theta)+phi);
        return x;
    }
    public double getY(){
        double a = robotRadius;
//        double theta2 = Math.toRadians(90)-centerTheta;
        double n9 = Math.toRadians(90);
        double d = Math.sqrt(l1*l1+a*a-(2*Math.abs(l1)*a*Math.cos(centerTheta+n9)));
        dy = d;
        double phi = Math.asin((a*Math.sin(centerTheta+n9))/d);
        double y = d*Math.cos(Math.toRadians(theta)-phi);
        return y;
    }


}
