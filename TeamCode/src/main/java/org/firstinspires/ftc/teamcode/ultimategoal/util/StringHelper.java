package org.firstinspires.ftc.teamcode.ultimategoal.util;

public class StringHelper {
    static StringBuilder s = new StringBuilder();

    public static String concat(Object ... objects){
        s.delete(0,s.length());

        for(Object o : objects){
            s.append(o.toString());
        }

        return s.toString();
    }
}