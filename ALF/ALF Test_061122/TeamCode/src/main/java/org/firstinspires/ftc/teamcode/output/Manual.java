package org.firstinspires.ftc.teamcode.output;

import static org.firstinspires.ftc.teamcode.commonFunctions.toggleVariable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ManualOutput", group = "Output")
public class Manual extends LinearOpMode {

    DcMotorEx liftOutput = null;
    Servo gripper = null;
    Servo transfer = null;

    double gripperPos = 0;
    double transferPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status:", "Initialized");
        telemetry.addData("Mechanism: ", "Output");
        telemetry.update();

        liftOutput = hardwareMap.get(DcMotorEx.class, "liftOutput");
        gripper = hardwareMap.get(Servo.class, "gripperOutput");
        transfer = hardwareMap.get(Servo.class, "transferOutput");

        liftOutput.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftOutput.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gripper.setPosition(0.5);
        transfer.setPosition(0.5);

        waitForStart();



        while(opModeIsActive()){
            if(gamepad1.a){
                liftOutput.setPower(gamepad1.left_trigger);
            }
            else if(gamepad1.b){
                liftOutput.setPower(-gamepad1.left_trigger);
            }
            else{
                liftOutput.setPower(0);
            }

            if(gamepad1.x){
                gripper.setPosition(0);
            }

            if(gamepad1.y){
                gripper.setPosition(1);
            }

            if(gamepad1.dpad_down){
                transfer.setPosition(0);
            }
            if(gamepad1.dpad_up){
                transfer.setPosition(1);
            }


            telemetry.addLine("Press left trigger to control speed.\na and b set direction.");
            telemetry.addData("Lift position: ", liftOutput.getCurrentPosition());
            telemetry.update();
        }
    }
}
