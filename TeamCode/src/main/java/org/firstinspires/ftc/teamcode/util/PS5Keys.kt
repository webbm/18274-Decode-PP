package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.gamepad.GamepadKeys

class PS5Keys {

    enum class Button(val xboxButton: GamepadKeys.Button) {
        SQUARE(GamepadKeys.Button.X),
        CROSS(GamepadKeys.Button.A),
        CIRCLE(GamepadKeys.Button.B),
        TRIANGLE(GamepadKeys.Button.Y),
        LEFT_BUMPER(GamepadKeys.Button.LEFT_BUMPER),
        RIGHT_BUMPER(GamepadKeys.Button.RIGHT_BUMPER),
        SHARE(GamepadKeys.Button.BACK),
        OPTIONS(GamepadKeys.Button.START),
        LSB(GamepadKeys.Button.LEFT_STICK_BUTTON),
        RSB(GamepadKeys.Button.RIGHT_STICK_BUTTON),
        DPAD_LEFT(GamepadKeys.Button.DPAD_LEFT),
        DPAD_RIGHT(GamepadKeys.Button.DPAD_RIGHT),
        DPAD_UP(GamepadKeys.Button.DPAD_UP),
        DPAD_DOWN(GamepadKeys.Button.DPAD_DOWN),
    }

    enum class Trigger(val xboxTrigger: GamepadKeys.Trigger) {
        LEFT_TRIGGER(GamepadKeys.Trigger.LEFT_TRIGGER),
        RIGHT_TRIGGER(GamepadKeys.Trigger.RIGHT_TRIGGER),
    }
}
