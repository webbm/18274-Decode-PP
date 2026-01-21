package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp(name = "Motor Test", group = "Test")
class MotorTest : LinearOpMode() {
    override fun runOpMode() {
        val rightFront = hardwareMap.get(DcMotorEx::class.java, "right_front")
        val rightBack = hardwareMap.get(DcMotorEx::class.java, "right_back")
        val leftFront = hardwareMap.get(DcMotorEx::class.java, "left_front")
        val leftBack = hardwareMap.get(DcMotorEx::class.java, "left_back")
        val leftFlywheel = hardwareMap.get(DcMotorEx::class.java, "left_flywheel")
        val rightFlywheel = hardwareMap.get(DcMotorEx::class.java, "right_flywheel")
        val intakeRight = hardwareMap.get(DcMotorEx::class.java, "intake_right")
        val intakeLeft = hardwareMap.get(DcMotorEx::class.java, "intake_left")

        val motors = listOf(
            rightFront,
            rightBack,
            leftFront,
            leftBack,
            leftFlywheel,
            rightFlywheel,
            intakeRight,
            intakeLeft,
        )

        motors.forEach { motor ->
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        waitForStart()

        while (opModeIsActive()) {
            rightFront.power = if (gamepad1.a) 1.0 else 0.0
            rightBack.power = if (gamepad1.b) 1.0 else 0.0
            leftFront.power = if (gamepad1.x) 1.0 else 0.0
            leftBack.power = if (gamepad1.y) 1.0 else 0.0
            leftFlywheel.power = if (gamepad1.dpad_left) 1.0 else 0.0
            rightFlywheel.power = if (gamepad1.dpad_right) 1.0 else 0.0
            intakeRight.power = if (gamepad1.dpad_up) 1.0 else 0.0
            intakeLeft.power = if (gamepad1.dpad_down) 1.0 else 0.0

            telemetry.addLine("Motor Test - gamepad1")
            telemetry.addData("A", "right_front")
            telemetry.addData("B", "right-back")
            telemetry.addData("X", "left_front")
            telemetry.addData("Y", "left_back")
            telemetry.addData("Dpad Left", "leftFlywheel")
            telemetry.addData("Dpad Right", "rightFlywheel")
            telemetry.addData("Dpad Up", "intake_Right")
            telemetry.addData("Dpad Down", "intake_Left")
            telemetry.update()
        }
    }
}
