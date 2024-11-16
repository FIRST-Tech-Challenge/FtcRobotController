package dev.aether.collaborative_multitasking

fun <K, V> MutableMap<K, V>.filterInPlace(predicate: (Map.Entry<K, V>) -> Boolean): List<K> {
    val collected: MutableList<K> = mutableListOf()
    for (pair in this.entries) {
        if (!predicate(pair)) {
            collected.add(pair.key)
        }
    }
    val deleted: MutableList<K> = mutableListOf()
    for (delete in collected) {
        val del = this.remove(delete)
        if (del != null) deleted.add(delete)
    }
    return deleted
}

fun <K, V> MutableMap<K, V>.filterInPlace(predicate: (K, V) -> Boolean) = filterInPlace { pair ->
    predicate(pair.key, pair.value)
}