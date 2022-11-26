package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class MotorTest extends LinearOpMode {
    DcMotorEx motor2 = null;
    DcMotorEx motor1;

    @Override
    public void runOpMode() throws InterruptedException {


        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");


        waitForStart();



        while(opModeIsActive()) {
            telemetry.addData("Motor 1 current: ", motor1.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Motor 2 Current: ", motor2.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
            if (gamepad1.a) {
                motor1.setPower(gamepad1.left_stick_y);
                motor2.setPower(gamepad1.left_stick_y);
            }
            else if(gamepad1.b){
                motor1.setPower(gamepad1.left_stick_y);
                motor2.setPower(gamepad1.left_stick_y);
            }
            else{
                motor1.setPower(0);
                motor2.setPower(0);
            }
        }
    }


    void extendTo(int pos){

    }

    void extendTo(double pos){
        int position = (int)pos;

    }
}
