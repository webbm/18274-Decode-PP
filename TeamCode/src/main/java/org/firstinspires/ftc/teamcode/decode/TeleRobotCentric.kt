package org.firstinspires.ftc.teamcode.decode

import android.R.attr.direction
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID
import org.firstinspires.ftc.teamcode.util.PS5Keys


/**
 * Test OpMode for complex movements including Travel Position
 */
@Disabled
@TeleOp(name = "Decode TeleOp")
class TeleRobotCentric : LinearOpMode() {

    enum class ShootingPosition(val min: Double, val middle: Double, val max: Double) {
        CLOSE(1900.0, 2200.0, 2800.0),
        FAR(2200.0, 2800.0, 3100.0);

        fun toggle(): ShootingPosition {
            return when (this) {
                CLOSE -> FAR
                FAR -> CLOSE
            }
        }
    }

    var shootingPosition = ShootingPosition.FAR

    private lateinit var follower: Follower
//    val startPose: Pose = Pose(6.5, 111.0, 0.0)

    lateinit var ffl: CRServo
    lateinit var ffr: CRServo
    lateinit var intakeRight: DcMotorEx
    lateinit var intakeLeft: DcMotorEx


    val config = ShooterConfig

    var shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)


//    val cyclePose: Pose = Pose(60.0, 98.0, Math.toRadians(270.0))
//    val cycleControlPose: Pose = Pose(65.0, 120.0, Math.toRadians(-90.0))

    override fun runOpMode() {

        val timer: ElapsedTime = ElapsedTime()

        follower = Constants.createFollower(hardwareMap)
        follower.update()

        // Initialize gamepad
        val driverGamepad = GamepadEx(gamepad1)
        val manipulatorGamepad = GamepadEx(gamepad2)
        val leftTriggerReader = TriggerReader(manipulatorGamepad, PS5Keys.Trigger.LEFT_TRIGGER.xboxTrigger)
        val rightTriggerReader = TriggerReader(manipulatorGamepad, PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger)

        // Initialize drive motors

        ffl = hardwareMap.get(CRServo::class.java, "feed_left").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }

        ffr = hardwareMap.get(CRServo::class.java, "feed_right").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

        intakeLeft = hardwareMap.get(DcMotorEx::class.java, "intake").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        intakeRight = hardwareMap.get(DcMotorEx::class.java, "intake").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

        shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)
        shooter.init(hardwareMap)

        follower.setStartingPose(Pose())

        telemetry.addData("Status", "Initialized")
        telemetry.addData("Controls", "Y button: Travel Position")
        telemetry.update()

        waitForStart()

        var shooterSpeed = 2000.0

        follower.startTeleopDrive(true)
        var reverseOn = false

        while (opModeIsActive()) {

            follower.update()

            shooter.update()

            val driveSpeedMultiplier = -1.0


            follower.setTeleOpDrive(
                driverGamepad.leftY,
                -driverGamepad.leftX,
                -driverGamepad.rightX,
                false // Robot Centric
            )
            follower.update()

           /* if (driverGamepad.wasJustPressed(PS5Keys.Button.LEFT_BUMPER.xboxButton) && !reverseOn) {
                follower.setTeleOpDrive(
                    gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    gamepad1.right_stick_x.toDouble(),
                    true // Robot Centric
                )
                follower.update()
                reverseOn = true
            }else if (driverGamepad.wasJustPressed(PS5Keys.Button.LEFT_BUMPER.xboxButton) && reverseOn){
                follower.setTeleOpDrive(
                    -gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble(),
                    gamepad1.right_stick_x.toDouble(),
                    true // Robot Centric
                )
                follower.update()
                reverseOn = false
            }*/

            if (manipulatorGamepad.getTrigger(PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger) >= 0.15){
               intakeLeft.power = .75
               intakeRight.power = .75
            }else if (manipulatorGamepad.getTrigger(PS5Keys.Trigger.LEFT_TRIGGER.xboxTrigger) >= 0.15){
                intakeLeft.power = -.75
                intakeRight.power = -.75
            }else {
                intakeLeft.power = 0.0
                intakeRight.power = 0.0
            }
            follower.update()

            if (driverGamepad.wasJustPressed(PS5Keys.Button.LEFT_BUMPER.xboxButton)) {
                follower.pose = Pose()
            }

            // Read gamepad inputs
            driverGamepad.readButtons()
            manipulatorGamepad.readButtons()
            leftTriggerReader.readValue()
            rightTriggerReader.readValue()

            shooter.update()

            /*if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.CROSS.xboxButton)) {
                shootingPosition = shootingPosition.toggle()
                shooterSpeed = shootingPosition.middle
            }
            if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.TRIANGLE.xboxButton)) {
                shooter.setTargetRpm(-2500.0)
            }

            if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_UP.xboxButton)){
                shooterSpeed = Range.clip(shooterSpeed + 100.0, shootingPosition.min, shootingPosition.max)
                shooter.setTargetRpm(shooterSpeed)
                shooter.update()
            }else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_DOWN.xboxButton)) {
                shooterSpeed = Range.clip(shooterSpeed - 100.0, shootingPosition.min, shootingPosition.max)
                shooter.setTargetRpm(shooterSpeed)
                shooter.update()
            }else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_LEFT.xboxButton)) {
                shooter.setTargetRpm(0.0)
            }else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_RIGHT.xboxButton)) {
                shooter.setTargetRpm(shooterSpeed)
            }
            shooter.update()*/

            if (manipulatorGamepad.isDown((PS5Keys.Button.RIGHT_BUMPER.xboxButton))){
                ffl.power = .6
                ffr.power = .6
            }else if (manipulatorGamepad.isDown((PS5Keys.Button.LEFT_BUMPER.xboxButton))) {
                ffl.power = -0.5
                ffr.power = -0.5
            }else {
                ffl.power = 0.0
                ffr.power = 0.0
            }

            shooter.update()
            follower.update()

            // Display telemetry
            telemetry.addData("Status", "Running")
            telemetry.addData("Shooter speed", shooterSpeed)
            telemetry.addData("Shooting Position", shootingPosition)
//            telemetry.addData("X", follower.pose.x)
//            telemetry.addData("Y", follower.pose.y)
//            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.pose.heading))
            telemetry.update()

            idle()

        }
    }
}
