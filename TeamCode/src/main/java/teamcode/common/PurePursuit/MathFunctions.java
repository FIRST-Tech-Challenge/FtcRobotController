package teamcode.common.PurePursuit;

public class MathFunctions {

    public static double angleWrap(double angle){
        // System.out.println("here out");
        angle = Math.toDegrees(angle);
        while(angle < -180){
            angle += 360;
        }

        while(angle > 180){
            angle -= 360;
        }

        return Math.toRadians(angle);
    }
}
