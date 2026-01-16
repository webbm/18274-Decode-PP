package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID
import kotlin.math.max

@TeleOp(name = "Flywheel Test", group = "Test")
class FlywheelTestOpMode : LinearOpMode() {

    private lateinit var leftFlywheel: DcMotorEx
    private lateinit var rightFlywheel: DcMotorEx

    override fun runOpMode() {
        val shooter = ShooterPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD, ShooterConfig.kF)

        leftFlywheel = hardwareMap.get(DcMotorEx::class.java, "leftFlywheel").apply {
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        rightFlywheel = hardwareMap.get(DcMotorEx::class.java, "rightFlywheel").apply {
            mode = DcMotor.RunMode.RUN_USING_ENCODER
            direction = DcMotorSimple.Direction.REVERSE
        }

        var targetRpm = 0.0
        var lastDpadUp = false
        var lastDpadDown = false

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        shooter.init(hardwareMap = hardwareMap)

        while (opModeIsActive()) {
            val dpadUpPressed = gamepad1.dpad_up && !lastDpadUp
            val dpadDownPressed = gamepad1.dpad_down && !lastDpadDown
            lastDpadUp = gamepad1.dpad_up
            lastDpadDown = gamepad1.dpad_down

            if (dpadUpPressed) {
                targetRpm += 50.0
            }
            if (dpadDownPressed) {
                targetRpm = max(0.0, targetRpm - 50.0)
            }

            shooter.setTargetRpm(targetRpm)
            shooter.update()

            telemetry.addData("Target RPM", "%.0f", targetRpm)
            telemetry.addData("Flywheel RPM", "%.0f", shooter.getFlywheelRpm())
            telemetry.addLine("Dpad Up/Down: +50/-50 RPM")
            telemetry.update()
        }
    }
}
