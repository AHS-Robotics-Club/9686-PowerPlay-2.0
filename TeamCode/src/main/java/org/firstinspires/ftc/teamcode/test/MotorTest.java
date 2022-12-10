package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Test Motor")
public class MotorTest extends CommandOpMode {
    private Motor testMotor;
    private GamepadEx gPad1;

    @Override
    public void initialize() {

        testMotor = new Motor(hardwareMap, "liftMotorLeft");

        gPad1 = new GamepadEx(gamepad1);

        gPad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new StartEndCommand(() -> testMotor.set(0.8), () -> testMotor.stopMotor()));
        gPad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new StartEndCommand(() -> testMotor.set(-0.8), () -> testMotor.stopMotor()));

    }

}

//Viggy laptop is so sexy, and so is he


//poopfart i love rishab