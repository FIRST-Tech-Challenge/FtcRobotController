package ftc.rouge.blacksmith.util

inline fun <T> runOnce(crossinline block: () -> T): () -> T {
    var value: T? = null

    return {
        value ?: block().also { value = it }
    }
}
