package android.util

// Fixes some thing with MockK wanting to log stuff but not finding a logger
object Log {
    @JvmStatic
    fun isLoggable(tag: String, level: Int): Boolean {
        return false
    }
}
