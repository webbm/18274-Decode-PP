package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Sample Kotlin (Linear)", group = "Examples")
class SampleLinearKotlinOpMode : LinearOpMode() {
    override fun runOpMode() {
        // Initialize hardware here if needed

        waitForStart()

        while (opModeIsActive()) {
            // Do nothing; just idle to keep the loop alive
            idle()
        }
    }
}

