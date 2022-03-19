package org.firstinspires.ftc.teamcode.src.utills;

public class JNIGetString {

    static {
        System.loadLibrary("JNIGetStr");
    }

    static {
        Double.toString(21);
    }

    public static native String stringFromJNI();

    public static native int intFromJNI();

    public static tmp getInstance() {
        tmp temp = new tmp();
        temp.val = 8;
        return temp;
    }

    public static native double extraCallFromJNI(tmp o);
}

class tmp {
    public double val = 0;
}
