package org.firstinspires.ftc.teamcode.intake;

import static org.firstinspires.ftc.teamcode.commonFunctions.toggleVariable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ManualIntake", group="Intake")
public class Manual extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Mechanism: ", "Intake");
        telemetry.update();

        double gripperPos = 0;
        double transferPos = 0;

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftIntake");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo transfer = hardwareMap.get(Servo.class, "transfer");
        waitForStart();

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive()){

            if(gamepad1.a){
                liftMotor.setPower(gamepad1.left_trigger);
            }
            else if(gamepad1.b){
                liftMotor.setPower(-gamepad1.left_trigger);
            }
            else{
                liftMotor.setPower(0);
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
            telemetry.addData("Lift position: ", liftMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
