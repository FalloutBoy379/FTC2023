package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


//@TeleOp
public class PrimitiveTesting extends LinearOpMode {

    DcMotorEx elevatorLeft, elevatorRight;
//    MotorEx elevatorLeft, elevatorRight;


//    MotorGroup elevator;

//    GamepadEx DriverJoystick = new GamepadEx(gamepad1);

//    ButtonReader positionPlusButton = new ButtonReader(
//            DriverJoystick, GamepadKeys.Button.A
//    );
//
//    ButtonReader positionMinusButton = new ButtonReader(
//            DriverJoystick, GamepadKeys.Button.B
//    );

    @Override
    public void runOpMode() throws InterruptedException {

        elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elevatorLeft = new MotorEx(hardwareMap, "frontEncoder", 28, 6000);
//        elevatorRight = new MotorEx(hardwareMap, "elevatorRight", 28, 6000);

//        elevatorRight.setInverted(true);

//        elevator = new MotorGroup(elevatorRight, elevatorLeft);

        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo slider = hardwareMap.get(Servo.class, "slider");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(36, -60, 0));


        waitForStart();

//        elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        double position = 0;

//        elevator.setRunMode(Motor.RunMode.PositionControl);
//        elevator.setPositionTolerance(10);
//        elevator.set(0);

        while(opModeIsActive()){
//            drive.update();


//            Pose2d robotPose = drive.getPoseEstimate();

//            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-robotPose.getHeading());

//            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));

//            telemetry.addData("X", robotPose.getX());
//            telemetry.addData("Y", robotPose.getY());
//            telemetry.addData("Heading", robotPose.getHeading());
//            if(gamepad1.x){
//                wrist.setPosition(0.75);
//            }
//            else if(gamepad1.right_stick_button){
//                wrist.setPosition(0.5);
//            }
//            else if(gamepad1.y){
//                wrist.setPosition(0);
//            }
//
//
//            if(gamepad1.right_bumper){
//                gripper.setPosition(1);
//            }
//            else if(gamepad1.left_bumper){
//                gripper.setPosition(0);
//            }

//            slider.setPosition((gamepad1.right_stick_y * 0.5)+0.5);
//            elevatorRight.setPower(gamepad1.right_stick_x);
//            elevatorLeft.setPower(gamepad1.right_stick_x);

            elevatorLeft.setPower(1);
            elevatorRight.setPower(1);
//            if(gamepad1.a){
//                position = 3530;
//            }
//            else if(gamepad1.b){
//                position = 250;
//            }

//            extendTo((int)position);
//            turret.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            telemetry.addData("Position Left", elevatorLeft.getCurrentPosition());
            telemetry.addData("Position Left", elevatorRight.getCurrentPosition());
            telemetry.addData("Current Left", elevatorLeft.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Current Right", elevatorRight.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();


        }




    }

    void extendTo(int pos){
        elevatorLeft.setTargetPosition(pos);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(1);
        elevatorRight.setTargetPosition(pos);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
    }
}
