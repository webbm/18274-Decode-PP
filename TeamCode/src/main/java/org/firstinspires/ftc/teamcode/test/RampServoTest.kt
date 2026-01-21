package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range

@Disabled
@TeleOp(name = "Ramp Servo Test", group = "Test")
class RampServoTest : LinearOpMode() {
    override fun runOpMode() {
        val leftRamp = hardwareMap.get(Servo::class.java, "left_ramp")
        val rightRamp = hardwareMap.get(Servo::class.java, "right_ramp")

        var leftPos = leftRamp.position
        var rightPos = rightRamp.position
        var lastA = false
        var lastB = false
        var lastDpadUp = false
        var lastDpadDown = false

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val aPressed = gamepad1.a && !lastA
            val bPressed = gamepad1.b && !lastB
            val dpadUpPressed = gamepad1.dpad_up && !lastDpadUp
            val dpadDownPressed = gamepad1.dpad_down && !lastDpadDown
            lastA = gamepad1.a
            lastB = gamepad1.b
            lastDpadUp = gamepad1.dpad_up
            lastDpadDown = gamepad1.dpad_down

            if (aPressed) {
                leftPos = if (leftPos < 0.5) 1.0 else 0.0
            }
            if (bPressed) {
                rightPos = if (rightPos < 0.5) 1.0 else 0.0
            }
            if (dpadUpPressed) {
                leftPos = Range.clip(leftPos + 0.05, 0.0, 1.0)
                rightPos = Range.clip(rightPos + 0.05, 0.0, 1.0)
            }
            if (dpadDownPressed) {
                leftPos = Range.clip(leftPos - 0.05, 0.0, 1.0)
                rightPos = Range.clip(rightPos - 0.05, 0.0, 1.0)
            }

            leftRamp.position = leftPos
            rightRamp.position = rightPos

            telemetry.addData("Left Ramp", "%.2f", leftPos)
            telemetry.addData("Right Ramp", "%.2f", rightPos)
            telemetry.addLine("A: toggle left (0/1), B: toggle right (0/1)")
            telemetry.addLine("Dpad Up/Down: adjust both by 0.05")
            telemetry.update()
        }
    }
}
