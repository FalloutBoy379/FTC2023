package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class TeleopTest extends OpMode {

    GamepadEx Joystick1;
    ButtonReader A, B, X, Y, UP, DOWN, LEFT, RIGHT, R1, L1, R3, L3, START, BACK;


    @Override
    public void init() {
        assignButtons(gamepad1);

    }


    @Override
    public void loop() {

    }


    public void assignButtons(Gamepad gamepadObj){
        Joystick1 = new GamepadEx(gamepadObj);
        A     = new ButtonReader(Joystick1, GamepadKeys.Button.A);
        B     = new ButtonReader(Joystick1, GamepadKeys.Button.B);
        X     = new ButtonReader(Joystick1, GamepadKeys.Button.X);
        Y     = new ButtonReader(Joystick1, GamepadKeys.Button.Y);
        UP    = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_UP);
        DOWN  = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_DOWN);
        LEFT  = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_LEFT);
        RIGHT = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_RIGHT);
        R1    = new ButtonReader(Joystick1, GamepadKeys.Button.RIGHT_BUMPER);
        L1    = new ButtonReader(Joystick1, GamepadKeys.Button.LEFT_BUMPER);
        R3    = new ButtonReader(Joystick1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        L3    = new ButtonReader(Joystick1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        START = new ButtonReader(Joystick1, GamepadKeys.Button.START);
        BACK  = new ButtonReader(Joystick1, GamepadKeys.Button.BACK);
    }

}
