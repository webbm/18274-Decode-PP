package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Disabled
@TeleOp(name = "Intake Test", group = "Test")
class IntakeTestOpMode : LinearOpMode() {
    override fun runOpMode() {
        val leftMotor = hardwareMap.get(DcMotorEx::class.java, "intake_Left")
        val rightMotor = hardwareMap.get(DcMotorEx::class.java, "intake_Right")

        leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        rightMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        while (opModeIsActive()) {
            val intake = gamepad2.right_bumper
            val outtake = gamepad2.left_bumper

            val power = when {
                intake -> 1.0
                outtake -> -1.0
                else -> 0.0
            }

            leftMotor.power = power
            rightMotor.power = power

            telemetry.addData("Mode", when {
                intake -> "Intake"
                outtake -> "Outtake"
                else -> "Idle"
            })
            telemetry.addData("Power", "%.2f", power)
            telemetry.update()
        }
    }
}
