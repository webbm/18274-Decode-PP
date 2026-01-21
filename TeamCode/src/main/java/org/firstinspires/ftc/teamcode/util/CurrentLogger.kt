package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter

class CurrentLogger(
    private val hardwareMap: HardwareMap,
    private val logFileName: String = "CurrentLogger.csv",
    private val logIntervalMs: Long = 250L
) {
    private val logFile: File = AppUtil.getInstance().getSettingsFile(logFileName)
    private val timer = ElapsedTime()
    private var lastLogMs = 0L

    private val hubs: List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)
    private val motors: List<DcMotorEx> = hardwareMap.getAll(DcMotorEx::class.java)
    private val crServos: List<CRServo> = hardwareMap.getAll(CRServo::class.java)
    private val servos: List<Servo> = hardwareMap.getAll(Servo::class.java)

    private val hubNames = hubs.map { nameForDevice(it) }
    private val motorNames = motors.map { nameForDevice(it) }
    private val crServoNames = crServos.map { nameForDevice(it) }
    private val servoNames = servos.map { nameForDevice(it) }

    fun start() {
        timer.reset()
        lastLogMs = 0L
        if (!logFile.exists()) {
            writeHeader()
        }
    }

    fun update() {
        val nowMs = timer.milliseconds().toLong()
        if (nowMs - lastLogMs < logIntervalMs) {
            return
        }
        lastLogMs = nowMs
        appendLine(buildLogLine(nowMs))
    }

    private fun writeHeader() {
        val header = StringBuilder()
            .append("time_ms,total_current_amps")

        hubNames.forEach { header.append(",hub_").append(it).append("_amps") }
        motorNames.forEach { header.append(",motor_").append(it).append("_amps") }
        crServoNames.forEach { header.append(",crservo_").append(it).append("_power") }
        servoNames.forEach { header.append(",servo_").append(it).append("_pos") }

        appendLine(header.toString())
    }

    private fun buildLogLine(timeMs: Long): String {
        val line = StringBuilder()
        var totalCurrent = 0.0

        hubs.forEach { module ->
            totalCurrent += module.getCurrent(CurrentUnit.AMPS)
        }

        line.append(timeMs).append(",").append(format(totalCurrent))

        hubs.forEach { module ->
            line.append(",").append(format(module.getCurrent(CurrentUnit.AMPS)))
        }
        motors.forEach { motor ->
            line.append(",").append(format(motor.getCurrent(CurrentUnit.AMPS)))
        }
        crServos.forEach { servo ->
            line.append(",").append(format(servo.power))
        }
        servos.forEach { servo ->
            line.append(",").append(format(servo.position))
        }

        return line.toString()
    }

    private fun appendLine(line: String) {
        try {
            BufferedWriter(FileWriter(logFile, true)).use { writer ->
                writer.append(line)
                writer.newLine()
            }
        } catch (e: Exception) {
            RobotLog.ee("CurrentLogger", e, "Failed to write %s", logFile.absolutePath)
        }
    }

    private fun nameForDevice(device: HardwareDevice): String {
        val rawName = hardwareMap.getNamesOf(device).firstOrNull() ?: device.deviceName
        return rawName.replace("[^A-Za-z0-9_]+".toRegex(), "_")
    }

    private fun format(value: Double): String {
        return String.format("%.3f", value)
    }
}
