@file:ConfigKt

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
        const T = λxy.x
        const F = λxy.y
        
        const Yes = λa.a
        const Not = λaxy.ayx
        
        const And = λab.aba
        const Or  = λab.aab
        
        const Leq = λab.ab $ `Not`b
        const Xor = λab.a(`Not`b)b 
        
        const Nand = λab.a(`Not`b)T
        const Nor  = λab.aF(`Not`b)
        
        return Nand(F)(F)('true')('false')
    """.trimIndent()
        .lamdafy()
        .also(::printWithSep)
        .expand1()
        .split("\n")
        .joinToString("\n", transform = String::test1)
        .also(::printWithSep)

    println("------------------------------------------------")

    val process = ProcessBuilder("node", "-e", "\"console.log((() => {$jstr})())\"")
        .start()

    process.inputStream.bufferedReader().use {
        it.readLine().also(::println)
    }

    process.waitFor()
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

fun String.test1(): String {
    if ('λ' !in this) {
        return this
    }

    val s = StringBuilder()

    var i = indexOf('λ')
    val arr1 = arrayOf('λ', '.')
    val parens = arrayOf('(', ')')

    s.append(substring(0, i))

    var numParens = 0

    while (i < length) {
        while (i < length && this[i] in arr1 || (this[i - 1] in arr1 && this[i].isLetter()))
            s.append( this[i++] )

        if (i == length)
            break

        val doParens = s.last() != '(' && s.last() !in arr1

        when {
            this[i] == '[' -> {
                while (this[++i] != ']') {
                    s.append( this[i] )
                }
            }

            this[i] == '`' -> {
                if (doParens) s.append("(")

                while (this[++i] != '`')
                    s.append(this[i])

                if (doParens) s.append(")")
            }

            this[i] == ' ' -> {}

            this[i] == '$' -> {
                s.append("(")
                numParens++
            }

            this[i] in parens || !doParens -> {
                s.append(this[i])
            }

            this[i] == '/' -> {
                s.append(' ')
                s.append(substring(i))
                break
            }

            else -> {
                s.append("(${this[i]})")
            }
        }
        i++
    }

    s.append(")".repeat(numParens))

    return s.toString()
        .replace(Regex("λ(\\w)\\.")) {
            val arg = it.groupValues[1]

            "$arg => "
        }
}

fun printWithSep(str: String) = println("------------------------------------------------\n$str")
