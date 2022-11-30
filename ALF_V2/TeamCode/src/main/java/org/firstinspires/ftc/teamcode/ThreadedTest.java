//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.gamepad.ButtonReader;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.teamcode.Mechanisms.Extension;
//import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
//
//public class ThreadedTest extends OpMode {
//
//
////    Thread extensionReadyThread = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            extension.extendTo(GRIPPINGPOS[0]);
//            intake.extendLift(GRIPPINGPOS[1]);
//            delayMs(1000);
//            extension.goInside();
//            extension.openGripper();
//            extensionReadyFlag = true;
//        }
//
//
//        public void delayMs(int ms){
//            try{Thread.sleep(ms);}
//            catch (InterruptedException ie){
//                Thread.currentThread().interrupt();
//            }
//        }
//    }).start();
//    GamepadEx Joystick1;
//    ButtonReader A, B, X, Y, UP, DOWN, LEFT, RIGHT, R1, L1, R3, L3, START, BACK;
//    Extension extension;
//    Intake intake;
//
//    final int[] GRIPPINGPOS = {35,140}; //Extension, Input
//
//    volatile boolean extensionReadyFlag;
//
//    {
//        extensionReadyFlag = false;
//    }
//
//    boolean transferConfigReadyFlag = false;
//    boolean[] generalFlags = {false,false,false};
//
//
//    @Override
//    public void init() {
//        extension = new Extension(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap, telemetry);
//        assignButtons(gamepad1);
//        extension.goInside();
//        extension.closeGripper();
//        intake.closeInputGripper();
//        intake.goOutside();
//    }
//
//    boolean zeroFlag = false;
//    boolean zeroFlag2 = false;
//    @Override
//    public void init_loop() {
//        telemetry.addData("Intake Sensor: ", Intake.homeSensor.isPressed());
//        if(!Intake.homeSensor.isPressed() && !zeroFlag2){
//            telemetry.addLine("Going to target");
//            intake.driveMotor(0.5);
//        }
//        else if(!zeroFlag2){
//            zeroFlag2 = true;
//            intake.driveMotor(0);
//            intake.reset();
//            intake.extendLift(0);
//        }
//        telemetry.addData("Extension Sensor: ", Extension.homingSensor.isPressed());
//        if(!Extension.homingSensor.isPressed() && !zeroFlag){
//            telemetry.addLine("Going to target");
//            extension.driveMotors(0.5);
//        }
//        else if(!zeroFlag){
//            zeroFlag = true;
//            extension.driveMotors(0);
//            extension.reset();
//            extension.extendTo(0);
//        }
//    }
//
//
//    @Override
//    public void loop() {
//
//    }
//
//    public void printStatement(String data){
//        telemetry.addLine(data);
//    }
//
//
//    public void assignButtons(Gamepad gamepadObj){
//        Joystick1 = new GamepadEx(gamepadObj);
//        A     = new ButtonReader(Joystick1, GamepadKeys.Button.A);
//        B     = new ButtonReader(Joystick1, GamepadKeys.Button.B);
//        X     = new ButtonReader(Joystick1, GamepadKeys.Button.X);
//        Y     = new ButtonReader(Joystick1, GamepadKeys.Button.Y);
//        UP    = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_UP);
//        DOWN  = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_DOWN);
//        LEFT  = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_LEFT);
//        RIGHT = new ButtonReader(Joystick1, GamepadKeys.Button.DPAD_RIGHT);
//        R1    = new ButtonReader(Joystick1, GamepadKeys.Button.RIGHT_BUMPER);
//        L1    = new ButtonReader(Joystick1, GamepadKeys.Button.LEFT_BUMPER);
//        R3    = new ButtonReader(Joystick1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
//        L3    = new ButtonReader(Joystick1, GamepadKeys.Button.LEFT_STICK_BUTTON);
//        START = new ButtonReader(Joystick1, GamepadKeys.Button.START);
//        BACK  = new ButtonReader(Joystick1, GamepadKeys.Button.BACK);
//    }
//}
