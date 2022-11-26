//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//@TeleOp
//public class YashTest extends LinearOpMode {
//
//    DcMotorEx leftMotor = null;
//    DcMotorEx rightMotor = null;
//    DcMotorEx liftMotor = null;
//    Servo gripper = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData("Status ; ", "Initialized");
//        telemetry.update();
//        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
//        rightMotor = hardwareMap.get(DcMotorEx.class, "right");
//        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
//        gripper = hardwareMap.get(Servo.class, "gripper");
//
//
//        waitForStart();
//        double position = 0;
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        while(opModeIsActive()){
//            double throttle = gamepad1.right_stick_x;
//            double turn = -gamepad1.left_stick_y;
//
//            double rightPower = throttle+turn;
//            double leftPower = throttle-turn;
//
//            leftMotor.setPower(leftPower);
//            rightMotor.setPower(rightPower);
//
//            if(gamepad1.right_bumper){
//                position = 450;
//            }
//            else if(gamepad1.left_bumper){
//                position = 2420;
//            }
//
//            extendTo((int)position);
//
//
//            if(gamepad1.a){
//                gripper.setPosition(0.75);
//            }
//            else if(gamepad1.y){
//                gripper.setPosition(0);
//            }
//            telemetry.addData("pos: ", position);
//            telemetry.update();
//        }
//    }
//
//    void extendTo(int pos){
//        liftMotor.setTargetPosition(pos);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setPower(1);
//    }
//}
