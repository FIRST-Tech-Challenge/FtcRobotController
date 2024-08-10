/*
Copyright (c) 2017 Robert Atkinson

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

import androidx.annotation.NonNull;
import androidx.annotation.StringRes;

import java.lang.reflect.Array;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;

/**
 * A collection of misfit utilities. They all need to go somewhere, and we can't
 * seem to find a better fit.
 */
@SuppressWarnings("WeakerAccess")
public class Misc
{
    public static final String TAG = "Misc";

    //----------------------------------------------------------------------------------------------
    // Strings
    //----------------------------------------------------------------------------------------------

    /** Formats the string using what in C# is called the 'invariant' culture */
    public static String formatInvariant(@NonNull String format, Object...args)
    {
        return String.format(Locale.ROOT, format, args);
    }
    public static String formatInvariant(@NonNull String format)
    {
        return format;
    }

    public static String formatForUser(@NonNull String format, Object...args)
    {
        return String.format(Locale.getDefault(), format, args);
    }
    public static String formatForUser(@NonNull String format)
    {
        return format;
    }

    public static String formatForUser(@StringRes int resId, Object...args)
    {
        return ""; // ### AppUtil.getDefContext().getString(resId, args);
    }
    public static String formatForUser(@StringRes int resId)
    {
        return ""; // ### AppUtil.getDefContext().getString(resId);
    }

    public static String encodeEntity(String string)
    {
        return encodeEntity(string, "");
    }
    public static String encodeEntity(String string, String rgchEscape)
    {
        StringBuilder builder = new StringBuilder();
        for (char ch : string.toCharArray())
        {
            switch (ch)
            {
                case '&':
                    builder.append("&amp;");
                    break;
                case '<':
                    builder.append("&lt;");
                    break;
                case '>':
                    builder.append("&gt;");
                    break;
                case '"':
                    builder.append("&quot;");
                    break;
                case '\'':
                    builder.append("&apos;");
                    break;
                default:
                    if (rgchEscape.indexOf(ch) >= 0)
                        builder.append(Misc.formatInvariant("&#x%x;", ch));
                    else
                        builder.append(ch);
                    break;
            }
        }
        return builder.toString();
    }
    public static String decodeEntity(String string)
    {
        StringBuilder builder = new StringBuilder();
        for (int ich = 0; ich < string.length(); ich++)
        {
            char ch = string.charAt(ich);
            if (ch == '&')
            {
                ich++;
                int ichFirst = ich;
                while (string.charAt(ich) != ';')
                {
                    ich++;
                }
                String payload = string.substring(ichFirst, ich-1);
                switch (payload)
                {
                    case "amp":
                        builder.append('&');
                        break;
                    case "lt":
                        builder.append('<');
                        break;
                    case "gt":
                        builder.append('>');
                        break;
                    case "quot":
                        builder.append('"');
                        break;
                    case "apos":
                        builder.append('\'');
                        break;
                    default:
                        if (payload.length() > 2 && payload.charAt(0) == '#' && payload.charAt(1) == 'x')
                        {
                            payload = "0x" + payload.substring(2);
                            int i = Integer.decode(payload);
                            builder.append((char)i);
                        }
                        else
                            throw illegalArgumentException("illegal entity reference");
                }
            }
            else
                builder.append(ch);
        }
        return builder.toString();
    }
    //----------------------------------------------------------------------------------------------
    // Math
    //----------------------------------------------------------------------------------------------

    public static long saturatingAdd(long x, long y)
    {
        if (x == 0 || y == 0 || (x > 0 ^ y > 0))
        {
            //zero+N or one pos, another neg = no problems
            return x + y;
        }
        else if (x > 0)
        {
            //both pos, can only overflow
            return Long.MAX_VALUE - x < y ? Long.MAX_VALUE : x + y;
        }
        else
        {
            //both neg, can only underflow
            return Long.MIN_VALUE - x > y ? Long.MIN_VALUE : x + y;
        }
    }
    public static int saturatingAdd(int x, int y)
    {
        if (x == 0 || y == 0 || (x > 0 ^ y > 0))
        {
            //zero+N or one pos, another neg = no problems
            return x + y;
        }
        else if (x > 0)
        {
            //both pos, can only overflow
            return Integer.MAX_VALUE - x < y ? Integer.MAX_VALUE : x + y;
        }
        else
        {
            //both neg, can only underflow
            return Integer.MIN_VALUE - x > y ? Integer.MIN_VALUE : x + y;
        }
    }

    public static boolean isEven(byte value)
    {
        return (value & 1) == 0;
    }
    public static boolean isEven(short value)
    {
        return (value & 1) == 0;
    }
    public static boolean isEven(int value)
    {
        return (value & 1) == 0;
    }
    public static boolean isEven(long value)
    {
        return (value & 1) == 0;
    }

    public static boolean isOdd(byte value)
    {
        return !isEven(value);
    }
    public static boolean isOdd(short value)
    {
        return !isEven(value);
    }
    public static boolean isOdd(int value)
    {
        return !isEven(value);
    }
    public static boolean isOdd(long value)
    {
        return !isEven(value);
    }

    public static boolean isFinite(double d)
    {
        return !Double.isNaN(d) && !Double.isInfinite(d);
    }

    public static boolean approximatelyEquals(double a, double b)
    {
        return approximatelyEquals(a, b, 1e-9);
    }

    public static boolean approximatelyEquals(double a, double b, double tolerance)
    {
        if (a==b) return true;  // zero and infinity are the important cases
        double error = b==0 ? Math.abs(a) : Math.abs(a/b-1.0); // pretty arbitrary
        return error < tolerance;
    }

    //----------------------------------------------------------------------------------------------
    // UUIDs
    //----------------------------------------------------------------------------------------------

    /** @see <a href="http://www.ietf.org/rfc/rfc4122.txt">UUID Spec</a> */
    public static UUID uuidFromBytes(byte[] rgb, ByteOrder byteOrder)
    {
        Assert.assertTrue(rgb.length == 16);

        ByteBuffer readBuffer = ByteBuffer.wrap(rgb).order(byteOrder);
        ByteBuffer writeBuffer = ByteBuffer.allocate(8).order(ByteOrder.BIG_ENDIAN);

        // There's funky byte ordering in the first eight bytes
        writeBuffer.putInt(readBuffer.getInt());
        writeBuffer.putShort(readBuffer.getShort());
        writeBuffer.putShort(readBuffer.getShort());
        writeBuffer.rewind();
        long mostSignificant = writeBuffer.getLong();

        // The remaining eight bytes are unordered
        writeBuffer.rewind();
        writeBuffer.put(readBuffer);
        writeBuffer.rewind();
        long leastSignificant = writeBuffer.getLong();

        return new UUID(mostSignificant, leastSignificant);
    }

    //----------------------------------------------------------------------------------------------
    // Arrays
    //----------------------------------------------------------------------------------------------

    public static boolean contains(byte[] array, byte value)
    {
        for (byte i : array)
        {
            if (i == value) return true;
        }
        return false;
    }

    public static boolean contains(short[] array, short value)
    {
        for (short i : array)
        {
            if (i == value) return true;
        }
        return false;
    }

    public static boolean contains(int[] array, int value)
    {
        for (int i : array)
        {
            if (i == value) return true;
        }
        return false;
    }

    public static boolean contains(long[] array, long value)
    {
        for (long i : array)
        {
            if (i == value) return true;
        }
        return false;
    }

    public static <T> T[] toArray(T[] contents, Collection<T> collection)
    {
        int s = collection.size();
        if (contents.length < s)
        {
            @SuppressWarnings("unchecked") T[] newArray = (T[]) Array.newInstance(contents.getClass().getComponentType(), s);
            contents = newArray;
        }
        int i = 0;
        for (T t : collection)
        {
            contents[i++] = t;
        }
        if (contents.length > s)
        {
            contents[s] = null;
        }
        return contents;
    }

    public static <T> T[] toArray(T[] contents, ArrayList<T> collection)
    {
        return collection.toArray(contents);
    }

    public static long[] toLongArray(Collection<Long> collection)
    {
        long[] result = new long[collection.size()];
        int i = 0;
        for (Long value: collection)
        {
            result[i++] = value;
        }
        return result;
    }

    public static int[] toIntArray(Collection<Integer> collection)
    {
        int[] result = new int[collection.size()];
        int i = 0;
        for (Integer value : collection)
        {
            result[i++] = value;
        }
        return result;
    }

    public static short[] toShortArray(Collection<Short> collection)
    {
        short[] result = new short[collection.size()];
        int i = 0;
        for (Short value : collection)
        {
            result[i++] = value;
        }
        return result;
    }

    public static byte[] toByteArray(Collection<Byte> collection)
    {
        byte[] result = new byte[collection.size()];
        int i = 0;
        for (Byte value: collection)
        {
            result[i++] = value;
        }
        return result;
    }

    //----------------------------------------------------------------------------------------------
    // Collections
    //----------------------------------------------------------------------------------------------

    public static <E> Set<E> intersect(Set<E> left, Set<E> right)
    {
        Set<E> result = new HashSet<>();
        for (E element : left)
        {
            if (right.contains(element))
            {
                result.add(element);
            }
        }
        return result;
    }

    //----------------------------------------------------------------------------------------------
    // Exceptions
    //----------------------------------------------------------------------------------------------

    public static IllegalArgumentException illegalArgumentException(String message)
    {
        return new IllegalArgumentException(message);
    }
    public static IllegalArgumentException illegalArgumentException(String format, Object...args)
    {
        return new IllegalArgumentException(formatInvariant(format, args));
    }
    public static IllegalArgumentException illegalArgumentException(Throwable throwable, String format, Object...args)
    {
        return new IllegalArgumentException(formatInvariant(format, args), throwable);
    }
    public static IllegalArgumentException illegalArgumentException(Throwable throwable, String message)
    {
        return new IllegalArgumentException(message, throwable);
    }

    public static IllegalStateException illegalStateException(String message)
    {
        return new IllegalStateException(message);
    }
    public static IllegalStateException illegalStateException(String format, Object...args)
    {
        return new IllegalStateException(formatInvariant(format, args));
    }
    public static IllegalStateException illegalStateException(Throwable throwable, String format, Object...args)
    {
        return new IllegalStateException(formatInvariant(format, args), throwable);
    }
    public static IllegalStateException illegalStateException(Throwable throwable, String message)
    {
        return new IllegalStateException(message, throwable);
    }

    public static RuntimeException internalError(String message)
    {
        return new RuntimeException("internal error:" + message);
    }
    public static RuntimeException internalError(String format, Object...args)
    {
        return new RuntimeException("internal error:" + formatInvariant(format, args));
    }
    public static RuntimeException internalError(Throwable throwable, String format, Object...args)
    {
        return new RuntimeException("internal error:" + formatInvariant(format, args), throwable);
    }
    public static RuntimeException internalError(Throwable throwable, String message)
    {
        return new RuntimeException("internal error:" + message, throwable);
    }
}

