package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range

@TeleOp(name = "Feed Servo Test", group = "Test")
class FeedServoTest : LinearOpMode() {
    private enum class Mode {
        FIXED,
        SWEEP,
        OSCILLATE
    }

    override fun runOpMode() {
        val ffl = hardwareMap.get(Servo::class.java, "feed_left")
        val ffr = hardwareMap.get(Servo::class.java, "feed_right")

        var leftPos = 0.5
        var rightPos = 0.5
        var leftDir = 1.0
        var rightDir = 1.0
        var mode = Mode.FIXED
        var lastX = false

        val step = 0.02
        val minPos = 0.0
        val maxPos = 1.0
        val centerPos = 0.5

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val feedForward = gamepad1.right_bumper
            val feedReverse = gamepad1.left_bumper
            val xPressed = gamepad1.x && !lastX
            lastX = gamepad1.x

            if (xPressed) {
                mode = when (mode) {
                    Mode.FIXED -> Mode.SWEEP
                    Mode.SWEEP -> Mode.OSCILLATE
                    Mode.OSCILLATE -> Mode.FIXED
                }
            }

            when (mode) {
                Mode.FIXED -> {
                    val target = when {
                        feedForward -> maxPos
                        feedReverse -> minPos
                        else -> centerPos
                    }
                    leftPos = target
                    rightPos = target
                }
                Mode.SWEEP -> {
                    val direction = when {
                        feedForward -> 1.0
                        feedReverse -> -1.0
                        else -> 0.0
                    }
                    if (direction != 0.0) {
                        leftPos = wrapPos(leftPos + direction * step, minPos, maxPos)
                        rightPos = wrapPos(rightPos + direction * step, minPos, maxPos)
                    }
                }
                Mode.OSCILLATE -> {
                    val direction = when {
                        feedForward -> 1.0
                        feedReverse -> -1.0
                        else -> 0.0
                    }
                    if (direction != 0.0) {
                        leftPos += leftDir * step * direction
                        rightPos += rightDir * step * direction
                        if (leftPos >= maxPos || leftPos <= minPos) leftDir *= -1.0
                        if (rightPos >= maxPos || rightPos <= minPos) rightDir *= -1.0
                        leftPos = Range.clip(leftPos, minPos, maxPos)
                        rightPos = Range.clip(rightPos, minPos, maxPos)
                    }
                }
            }

            ffl.position = leftPos
            ffr.position = rightPos

            telemetry.addData("Mode", mode)
            telemetry.addData("Left Pos", "%.2f", leftPos)
            telemetry.addData("Right Pos", "%.2f", rightPos)
            telemetry.addLine("Right bumper: forward, Left bumper: reverse")
            telemetry.addLine("X: cycle mode (Fixed/Sweep/Oscillate)")
            telemetry.update()
        }
    }

    private fun wrapPos(value: Double, minPos: Double, maxPos: Double): Double {
        val range = maxPos - minPos
        var v = value
        while (v > maxPos) v -= range
        while (v < minPos) v += range
        return v
    }
}
