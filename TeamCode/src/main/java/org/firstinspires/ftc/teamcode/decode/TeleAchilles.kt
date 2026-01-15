package org.firstinspires.ftc.teamcode.decode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "TeleAchilles")
class TeleAchilles : LinearOpMode() {
    private lateinit var follower: Follower

    override fun runOpMode() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose())
        follower.update()

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        follower.startTeleopDrive(true)

        while (opModeIsActive()) {
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                false // Field centric
            )
            follower.update()
            idle()
        }
    }
}
