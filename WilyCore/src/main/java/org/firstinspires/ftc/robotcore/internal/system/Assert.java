/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.internal.system;

/**
 * {@link Assert} is a utility class for assertions that generates an exception which
 * gets written to the log, but then continues on with the application. The write to the
 * log gives us notices that something is amiss that needs addressing, but continuing with
 * the app might, for example, allow a robot to continue on in a match rather than aborting
 * in the middle, depending on the nature of the failure.
 */
public class Assert
{
    public static final String TAG = "Assert";

    public static void assertTrue(boolean value)
    {
        if (!value)
        {
            assertFailed();
        }
    }

    public static void assertFalse(boolean value)
    {
        if (value)
        {
            assertFailed();
        }
    }

    public static void assertNull(Object value)
    {
        if (value != null)
        {
            assertFailed();
        }
    }

    public static void assertNotNull(Object value)
    {
        if (value == null)
        {
            assertFailed();
        }
    }

    public static void assertEquals(int expected, int actual)
    {
        if (expected != actual)
        {
            assertFailed();
        }
    }

    //----------------------------------------------------------------------------------------------

    public static void assertTrue(boolean value, String format, Object... args)
    {
        if (!value)
        {
            assertFailed(format, args);
        }
    }

    public static void assertFalse(boolean value, String format, Object... args)
    {
        if (value)
        {
            assertFailed(format, args);
        }
    }

    public static void assertNull(Object value, String format, Object... args)
    {
        if (value != null)
        {
            assertFailed(format, args);
        }
    }

    public static void assertNotNull(Object value, String format, Object... args)
    {
        if (value == null)
        {
            assertFailed(format, args);
        }
    }

    //----------------------------------------------------------------------------------------------

    public static void assertFailed()
    {
        try {
            throw new RuntimeException("assertion failed");
        }
        catch (Exception e)
        {
            // ### RobotLog.aa(TAG, e, "assertion failed");
        }
    }

    public static void assertFailed(String format, Object[] args)
    {
        String message = String.format(format, args);
        String banner  = "assertion failed: " + message;
        try {
            throw new RuntimeException(banner);
        }
        catch (Exception e)
        {
            // ### RobotLog.aa(TAG, e, banner);
        }
    }
}
