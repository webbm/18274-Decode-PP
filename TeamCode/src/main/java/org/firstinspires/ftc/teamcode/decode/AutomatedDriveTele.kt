package org.firstinspires.ftc.teamcode.decode

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.FuturePose
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.HeadingInterpolator.FutureDouble
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import java.util.function.Supplier

@Configurable
@TeleOp
@Disabled
class AutomatedDriveTele : OpMode() {
     lateinit var follower: Follower
     var automatedDrive = false
     lateinit var pathChain: Supplier<PathChain?>
     lateinit var telemetryM: TelemetryManager
     var slowMode = false
     var slowModeMultiplier = 0.5

    var startingPose = Pose(24.0, 24.0, Math.toRadians(0.0))

    override fun init() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startingPose)
        follower.update()
        telemetryM = PanelsTelemetry.telemetry
        pathChain = Supplier {
            follower.pathBuilder() //Lazy Curve Generation
                .addPath(Path(BezierLine(FuturePose { follower.getPose() }, Pose(-45.0, -98.0))))
                .setHeadingInterpolation(
                    HeadingInterpolator.linearFromPoint(
                        FutureDouble { follower.getHeading() },
                        Math.toRadians(45.0),
                        0.8
                    )
                )
                .build()
        }
    }

    override fun start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive()
    }

    override fun loop() {
        //Call this once per loop
        follower.update()
        telemetryM.update()
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                true // Robot Centric
            )
            else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                true // Robot Centric
            )
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get())
            automatedDrive = true
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive()
            automatedDrive = false
        }
        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode
        }
        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25
        }
        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25
        }
        telemetryM.debug("position", follower.pose)
        telemetryM.debug("velocity", follower.velocity)
        telemetryM.debug("automatedDrive", automatedDrive)
    }

}