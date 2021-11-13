package org.firstinspires.ftc.teamcode.Tests.TestsUtil;

public class Test {
    private static boolean assertCase(Object first, Object second) {
        try {
            return first.equals(second);
        } catch (Exception e) {
            return first.toString().equals(second.toString());
        }
    }

    public static boolean Assert(Object first, Object second) throws AssertException {
        if (!assertCase(first, second)) {
            throw new AssertException(first, second);
        }
        return true;
    }
}
