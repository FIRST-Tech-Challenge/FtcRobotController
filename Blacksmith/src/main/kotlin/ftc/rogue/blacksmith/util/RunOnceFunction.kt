package ftc.rogue.blacksmith.util

/**
 * runs once. yes. go check the blacksmith docs.
 */
inline fun <T> runOnce(crossinline block: () -> T): () -> T {
    var value: T? = null

    return {
        value ?: block().also { value = it }
    }
}
