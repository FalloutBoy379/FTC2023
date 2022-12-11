package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(group = "drive")

public class tank_code extends LinearOpMode {
    double PULLEY_DIAMETER = 30.00; //mm
    double CIRCUMFERENCE = PULLEY_DIAMETER * Math.PI;
    double GEAR_RATIO = 18.90;
    double COUNT_PER_REV = GEAR_RATIO * 28;
    double REV_PER_COUNT = 1/COUNT_PER_REV;
    double MM_PER_COUNT = REV_PER_COUNT * CIRCUMFERENCE;
    double COUNT_PER_MM = 1/ MM_PER_COUNT;

    DcMotorEx lift;
    DcMotorEx motorL;
    DcMotorEx motorR;
    RevColorSensorV3 colorSensor;
    private double triggerDistance = 45;

    @Override
    public void runOpMode() throws InterruptedException {

        int pos =0;
        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);
        motorL = hardwareMap.get(DcMotorEx.class,"leftRear");
        motorR = hardwareMap.get(DcMotorEx.class,"rightRear");
        motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.get(DcMotorEx.class, "lifter");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor");
//        lift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        boolean gripFlag = false;
        while (!isStopRequested()) {

            telemetry.addData("Distance", colorSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Red: ", colorSensor.getNormalizedColors().red);
            telemetry.addData("Blue: ", colorSensor.getNormalizedColors().blue);
            telemetry.addData("Blue: ", colorSensor.getNormalizedColors().green);

            if(colorSensor.getDistance(DistanceUnit.MM)<triggerDistance && gripping.servo1.getPosition() <= 0.1 && lift.getCurrentPosition() < 100){
                gripFlag = true;
            }

            if (gamepad1.dpad_up) {
                pos=4820;                 // upper pole
            }
            if (gamepad1.right_bumper) {
                pos=3470;                 // middle pole
            }
            if (gamepad1.left_bumper) {
                pos=2000;                 // lower pole
            }
            if (gamepad1.dpad_down) {
                pos=0;
//                gripping.setservo(gripping.servo1, 0);
//                lift.setPower(0.6);
//                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.left_stick_button) {
//                lift.setTargetPosition(500);
                pos=500;
//                lift.setPower(0.4);
//                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if (gamepad1.x) {   //release
                gripping.setservo(gripping.servo1, 0);
                gripFlag = false;
            }
            if (gamepad1.b  || gripFlag) {      //grip
                gripping.setservo(gripping.servo1, 1);

//                 pos = 500;
//                setLiftPosition(pos);
            }
            if(gamepad1.y)
            {
                pos+=10;               //manual up
            }
            else if(gamepad1.a)
            {
                pos-=10;               //manual down
            }

            setLiftPosition(pos);

            telemetry.addData("lift", lift.getCurrentPosition());
//            telemetry.addData("maxspeed", maxspeed);
            telemetry.addData("lift current" , lift.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            double turn = (gamepad1.right_stick_x*0.5);
            double throttle = Math.pow(gamepad1.left_stick_y,3);
            double rightspeed = throttle + turn;
            double leftspeed = throttle - turn;

            motorL.setPower(leftspeed);
            motorR.setPower(rightspeed);
        }
    }

    public void gotoMM(double pos){
        setPosition(lift, (int)(pos * COUNT_PER_MM));
    }
    public void setPosition(DcMotorEx lift1, int pos)
    {
        lift1.setTargetPosition(pos);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(lift1.getTargetPosition() - lift1.getCurrentPosition() >= 0){
            lift.setPower(1);
        }
        else if(lift1.getTargetPosition() - lift1.getCurrentPosition() < 0) {
            lift1.setPower(0.7);
        }
    }
    public void setLiftPosition(double pos)
    {

        setPosition(lift, (int)pos);
    }

}