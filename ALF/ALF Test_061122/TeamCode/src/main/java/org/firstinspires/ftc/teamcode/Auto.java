package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="AutoTest")
public class Auto extends LinearOpMode {

    boolean startSequenceFlag = false;
    DcMotorEx liftIntake = null;
    Servo gripperIntake = null;
    Servo transferIntake = null;

    DcMotorEx liftOutput = null;
    Servo gripperOutput = null;
    Servo transferOutput = null;




    @Override
    public void runOpMode() throws InterruptedException {
        liftIntake = hardwareMap.get(DcMotorEx.class, "liftIntake");
        gripperIntake = hardwareMap.get(Servo.class, "gripperIntake");
        transferIntake = hardwareMap.get(Servo.class, "transferIntake");

        liftOutput = hardwareMap.get(DcMotorEx.class, "liftOutput");
        gripperOutput = hardwareMap.get(Servo.class, "gripperOutput");
        transferOutput = hardwareMap.get(Servo.class, "transferOutput");

        liftOutput.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        closeInputGripper();

        waitForStart();


        while(opModeIsActive()) {


        }
    }


    void setOutputPos(int pos){
        liftOutput.setTargetPosition(pos);
        liftOutput.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftOutput.setPower(1);
    }

    void setIntakePos(int pos){
        liftIntake.setTargetPosition(pos);
        liftIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftIntake.setPower(1);
    }

    void closeInputGripper(){
        gripperIntake.setPosition(1);
    }

    void openInputGripper(){
        gripperIntake.setPosition(0);
    }

    void closeOutputGripper(){
        gripperOutput.setPosition(1);
    }

    void openOutputGripper(){
        gripperOutput.setPosition(0.7);
    }
}
