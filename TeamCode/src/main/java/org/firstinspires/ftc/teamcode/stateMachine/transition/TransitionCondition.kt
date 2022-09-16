package robotuprising.lib.system.statemachine.transition

fun interface TransitionCondition {
    fun shouldTransition(): Boolean
}
