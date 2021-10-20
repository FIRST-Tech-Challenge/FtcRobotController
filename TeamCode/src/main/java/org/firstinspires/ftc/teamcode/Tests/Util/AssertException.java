package org.firstinspires.ftc.teamcode.Tests.Util;

public class AssertException extends Exception {
    public AssertException (Object first, Object second)
    {
        super(first.toString() + " is not equal to " + second.toString());
    }
}
