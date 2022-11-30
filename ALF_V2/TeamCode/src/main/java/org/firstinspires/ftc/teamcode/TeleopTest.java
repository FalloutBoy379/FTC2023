package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;

import java.util.InputMismatchException;

@TeleOp
public class TeleopTest extends OpMode {

    GamepadEx Joystick1;
    ButtonReader A, B, X, Y, UP, DOWN, LEFT, RIGHT, R1, L1, R3, L3, START, BACK;
    Extension extension;
    Intake intake;

    final int[] GRIPPINGPOS = {35,140}; //Extension, Input

    boolean extensionReadyFlag = false;
    boolean transferConfigReadyFlag = false;
    boolean[] generalFlags = {false,false,false};


    ElapsedTime[] timer = {new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS),
            new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS),
            new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS),
            new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)};




    @Override
    public void init() {
        timer[0].reset();
        extension = new Extension(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        assignButtons(gamepad1);
        extension.goInside();
        extension.closeGripper();
        intake.closeInputGripper();
        intake.goOutside();

    }

    boolean zeroFlag = false;
    boolean zeroFlag2 = false;
    @Override
    public void init_loop() {
        telemetry.addData("Intake Sensor: ", Intake.homeSensor.isPressed());
        if(!Intake.homeSensor.isPressed() && !zeroFlag2){
            telemetry.addLine("Going to target");
            intake.driveMotor(0.5);
        }
        else if(!zeroFlag2){
            zeroFlag2 = true;
            intake.driveMotor(0);
            intake.reset();
            intake.extendLift(0);
        }
        telemetry.addData("Extension Sensor: ", Extension.homingSensor.isPressed());
        if(!Extension.homingSensor.isPressed() && !zeroFlag){
            telemetry.addLine("Going to target");
            extension.driveMotors(0.5);
        }
        else if(!zeroFlag){
            zeroFlag = true;
            extension.driveMotors(0);
            extension.reset();
            extension.extendTo(0);
        }
    }

    @Override
    public void loop() {
        if(A.isDown()){
            intake.goInside();
//            extension.closeGripper();
        }
        else if(B.isDown()){
            intake.goOutside();
        }

        if(X.isDown()){
            intake.closeInputGripper();
        }

        if(Y.isDown()){
            intake.openInputGripper();
        }

        if(L1.isDown()){
            sequence1();
        }
//        extension.disableMotors();
//intake.disableMotors();
//        extension.extendTo(extension.targetMillimeters);
//        intake.extendLift(Intake.targetMillimeters_input);
        telemetry.addData("Time: ", timer[0].milliseconds());
        telemetry.addData("Target Extension: ", extension.targetMillimeters);
        telemetry.addData("Target input: ", Intake.targetMillimeters_input);
        extension.periodic();
        intake.periodic();

    }


    public void sequence1(){
        printStatement("Executing Sequence 1");


            new Thread(new Runnable() {
                @Override
                public void run() {
                    extension.extendTo(GRIPPINGPOS[0]);
                    intake.extendLift(GRIPPINGPOS[1]);
                    sleep(1000);
                    extension.goInside();
                    extension.openGripper();
                    Extension.targetMillimeters = GRIPPINGPOS[0];
                    extensionReadyFlag = true;
                    printStatement("Transfer Configuration stage 1 Entered");
                    intake.openInputGripper();
                    Intake.targetMillimeters_input = 30;
                    intake.extendLift(Intake.targetMillimeters_input);
                    delayMs(1000);
                    intake.closeInputGripper();
                    delayMs(1000);
                    Intake.targetMillimeters_input = GRIPPINGPOS[1];
                    intake.extendLift(Intake.targetMillimeters_input);
                    delayMs(500);
                    intake.goInside();
                    delayMs(900);
                    extension.closeGripper();
                    delayMs(500);
                    intake.openInputGripper();
                    transferConfigReadyFlag = true;
                    extension.extendTo(540);
                    Extension.targetMillimeters = 540;
                    delayMs(1000);
                    extension.goOutside();
                    delayMs(1000);
                    extension.openGripper();
                    delayMs(500);
                    extension.extendTo(GRIPPINGPOS[0]);

                }

                public void delayMs(int ms){
                    try{Thread.sleep(ms);}
                    catch (InterruptedException ie){
                        Thread.currentThread().interrupt();
                    }
                }
            }).start();

    }




    public void printStatement(String data){
        telemetry.addLine(data);
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
