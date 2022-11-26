package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class servoTest extends LinearOpMode {

    Servo motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(Servo.class, "motor");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                motor.setPosition(1);
            }
            else if(gamepad1.b){
                motor.setPosition(0);
            }
            else{
                motor.setPosition(0.5);
            }
        }
    }
}
