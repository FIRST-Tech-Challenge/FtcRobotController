package teamCode;

import androidx.annotation.NonNull;

import java.util.function.BooleanSupplier;

public class Logic
{
    public static class OpModeType
    {
        public static String opMode;
    }

    public static final class DriveTrainConstants
    {

    }
    public static final class WaitClass
    {
        /**
         * @PARAMS: Wait until parameter event is true.
         */
        public static void wait(@NonNull BooleanSupplier condition)
        {
            while(!condition.getAsBoolean())
            {
            }
        }
    }
}
