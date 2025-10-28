package org.firstinspires.ftc.teamcode.tuners

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID

@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
@Configurable
class ShooterPIDFTuner: OpMode() {



    val config = ShooterConfig
    var shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)

    override fun init() {
        shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)
        shooter.init(hardwareMap)
    }

    override fun loop() {
        shooter.setTargetRpm(config.targetRpm)
        shooter.update()

        telemetry.addData("Target RPM", config.targetRpm)
        telemetry.addData("kP", config.kP)
        telemetry.addData("kI", config.kI)
        telemetry.addData("kD", config.kD)
        telemetry.addData("kF", config.kF)
        telemetry.addData("rightFlywheel", shooter.flywheel())
        telemetry.update()
    }
}