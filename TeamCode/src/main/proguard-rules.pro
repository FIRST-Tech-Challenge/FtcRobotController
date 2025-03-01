# Remove all Log.d calls from TeamCode
-assumenosideeffects class android.util.Log {
    public static int d(...);
}

-keep class org.jetbrains.annotations.** { *; }
-keep class org.slf4j.** { *; }

