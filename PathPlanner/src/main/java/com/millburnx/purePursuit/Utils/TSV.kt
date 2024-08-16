package com.millburnx.purePursuit.Utils

import java.io.File

class TSV {
    companion object {
        fun read(file: File): List<List<String>> {
            return parse(file.readText())
        }

        fun bufferedRead(file: File): List<List<String>> {
            return parse(file.bufferedReader().use { it.readText() })
        }

        fun parseRow(tsv: String): List<String> {
            return tsv.split("\t")
        }

        fun parse(tsv: String): List<List<String>> {
            return tsv.split("\n").map { parseRow(it) }
        }

        fun stringify(data: List<List<String>>): String {
            return data.joinToString("\n") { it.joinToString("\t") }
        }

        fun write(file: File, data: List<List<String>>) {
            file.writeText(stringify(data))
        }

        fun bufferedWrite(file: File, data: List<List<String>>) {
            file.bufferedWriter().use { it.write(stringify(data)) }
        }
    }
}