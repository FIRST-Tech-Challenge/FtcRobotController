@file:ConfigKt
@file:Suppress("SpellCheckingInspection")

package org.firstinspires.ftc.teamcodekt

import ftc.rogue.blacksmith.annotations.ConfigKt
import ftc.rogue.blacksmith.util.kt.clamp

@ConfigKt
object MoerTeasting {
    var hi = 3
}

var HELLO = 4.0.toInt().coerceAtLeast(2).clamp<Int>(0, 5)
var WHELLO = "4"

fun main() {
    val jstr = """
        T := λxy.x
        F := λxy.y
        
        YES := λa.a
        NOT := λaxy.ayx
        
        AND := λab.aba
        OR  := λab.aab
        
        XNOR := λab.ab﹩~b
        XOR  := λab.a(~b)b
        
        NAND := λab.a(~b)T
        NOR  := λab.aF(~b)
        
        // ---------------------------------------------------
        
        ZERO  := λpfa.a
        ONE   := λp.p(λfa.fa)(λpfa.a)
        TWO   := λp.p(λfa.f(fa))(λp.p(λfa.fa)(λpfa.a))
        THREE := SUCC(TWO)
        
        GET  := λnfa.nTfa
        PRED := λnfa.nFfa
        SUCC := λnp.p(λfa.f(nTfa))(n)
        
        ADD := λxy.y(T)SUCCx
        MUL := λxy.y(T)(ADDx)ZERO
        POW := λxy.y(T)(MULx)ONE
        
        // ---------------------------------------------------
        
        function count(f) {
            return f(a => a + 1)(0)
        }
        
        // ---------------------------------------------------
        
        return [
            count﹩GET﹩SUCC(ZERO)
           ,count﹩GET﹩ONE
           ,count﹩GET﹩PRED(TWO)
           ,count﹩GET﹩ADD(THREE)(TWO)
           ,count﹩GET﹩MUL(THREE)(TWO)
           ,count﹩GET﹩POW(THREE)(THREE)
        ].join('\n')
    """.trimIndent()
        .expand3()
        .also(::printWithSep)
        .expand1()
        .expand4()
        .expand2(2)
        .also(::printWithSep)
        .split("\n")
        .joinToString("\n", transform = String::test1)
        .also(::printWithSep)

    println("------------------------------------------------")

    val process = ProcessBuilder("node", "-e", "\"console.log((() => {$jstr})())\"")
        .start()

    process.inputStream.bufferedReader().use {
        it.readLines().forEach(::println)
    }

    process.errorStream.bufferedReader().use {
        it.readLines().forEach(::println)
    }

    process.waitFor()
}

fun String.expand2(expandTImes: Int): String {
    var acc = this

    for (i in 0..expandTImes) {
        Regex("(.*)const ([A-Z]+)\\s+=\\s+(.*)\\s+")
            .findAll(this)
            .filter {
                "//" !in it.groupValues[1]
            }
            .forEach {
                val name = it.groupValues[2]
                val replacement = it.groupValues[3]

                val regex = "(?<!const |[A-Z])$name(?![A-Z])".toRegex()

                acc = acc.replace(regex) {
                    val a = it.range.first - 1
                    val b = it.range.last + 1

                    if (a > 0 && acc[a] == '(' && b < acc.length && acc[b] == ')') {
                        replacement
                    } else {
                        "($replacement)"
                    }
                }
            }
    }

    return acc
}

fun String.expand3() = this
    .replace("~", "NOT")
    .split('\n')
    .joinToString("\n") {
        var numParens = 0

        it.replace("\\s*﹩\\s*".toRegex()) {
            numParens++
            "("
        } + ")".repeat(numParens)
    }

fun String.lamdafy() =
    replace('\\', 'λ')

fun String.expand1() =
    replace(Regex("λ(\\w+)")) {
        val args = it.groupValues[1]

        "λ" + args
            .split("")
            .filter(String::isNotBlank)
            .joinToString(".λ")
    }

fun String.expand4() =
    replace("([A-Z]+)(\\s+):=".toRegex()) {
        "const ${it.groupValues[1]}${it.groupValues[2]}="
    }

fun String.test1(): String {
    if ('λ' !in this || this.startsWith("//")) {
        return this
    }

    operator fun StringBuilder.plusAssign(other: Any) {
        this.append(other)
    }

    val s = StringBuilder()

    var i = indexOf('λ')
    val arr1 = arrayOf('λ', '.')
    val parens = arrayOf('(', ')')

    s += substring(0, i)

    while (i < length) {
        while (i < length && this[i] in arr1 || (this[i - 1] in arr1 && this[i].isLetter()))
            s += this[i++]

        if (i == length)
            break

        val doParens = s.last() !in arr1 + '('

        when {
            this[i] == '[' -> {
                while (this[++i] != ']') {
                    s += this[i]
                }
            }

            this[i].isUpperCase() -> {
                if (doParens) s += "("

                while (i < length && this[i].isUpperCase())
                    s += this[i++]

                if (doParens) s += ")"
                i--
            }

            this[i] == '\'' -> {
                s += '\''

                while (this[++i] != '\'')
                    s += this[i]

                s += '\''
            }

            this[i] == '`' -> {
                while (this[++i] != '`')
                    s += this[i]
            }

            this[i] == ' ' -> {}

            this[i] in parens + ',' || !doParens -> {
                s += this[i]
            }

            this[i] == '/' -> {
                s += " ${substring(i)}"
                break
            }

            else -> {
                s += "(${this[i]})"
            }
        }
        i++
    }

    return s.toString()
        .replace(Regex("λ(\\w)\\.")) {
            val arg = it.groupValues[1]
            "$arg => "
        }
}

fun printWithSep(str: String) = println("------------------------------------------------\n$str")
