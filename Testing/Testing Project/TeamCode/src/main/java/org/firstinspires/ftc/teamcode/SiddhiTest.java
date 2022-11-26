//package org.firstinspires.ftc.teamcode;
//
//import android.media.JetPlayer;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//public class SiddhiTest extends LinearOpMode {
//    Servo servo1 = null;
//    Servo servo2 = null;
//    Servo servo3 = null;
//
//    double pos1 = 0, pos2 = 0, pos3 = 0;
//    double prevJoy1 = 0, prevJoy2 = 0;
//    public void runOpMode(){
//        servo1 = hardwareMap.get(Servo.class, "servo1");
//        servo2 = hardwareMap.get(Servo.class, "servo2");
//        servo3 = hardwareMap.get(Servo.class, "servo3");
//        waitForStart();
//
//        while(opModeIsActive()){
//            double Joy1 = gamepad1.left_stick_y;
//            double Joy2 = gamepad1.right_stick_y;
//            if(Joy1 > prevJoy1){
//                pos1 = pos1 + 0.05;
//            }
//            else if(Joy1 < prevJoy1){
//                pos1 = pos1 - 0.05;
//            }
//
//            if(Joy2 > prevJoy2){
//                pos2 = pos2 + 0.05;
//            }
//            else if(Joy2 < prevJoy2){
//                pos2 = pos2 - 0.05;
//            }
//
//            if(gamepad1.a){
//                pos3 = 1;
//            }
//            else if(gamepad1.b){
//                pos3 = 0;
//            }
//
//            pos1 = Range.clip(pos1, 0, 1);
//            pos2 = Range.clip(pos2, 0, 1);
//            pos3 = Range.clip(pos3, 0, 1);
//
//            servo3.setPosition(pos3);
//            servo2.setPosition(pos2);
//            servo1.setPosition(pos1);
//
//
//        }
//    }
//}
