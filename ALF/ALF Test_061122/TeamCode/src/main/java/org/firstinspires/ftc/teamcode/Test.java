package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intake.Input;
import org.firstinspires.ftc.teamcode.output.Output;


@TeleOp
public class Test extends LinearOpMode {

    Output.Wrist outputWrist = null;
    Input.Wrist intakeWrist = null;
    Input.Gripper inputGripper = null;
    Output.Gripper outputgripper = null;
    Output.Extension extension = null;
    Input.Lift lift = null;


    double maxPos = 1600;
    double minPos = 190;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status: ", "Intialized");

        outputWrist = new Output.Wrist(hardwareMap);
        intakeWrist = new Input.Wrist(hardwareMap);
        inputGripper = new Input.Gripper(hardwareMap);
        outputgripper = new Output.Gripper(hardwareMap);
        extension = new Output.Extension(hardwareMap);
        lift = new Input.Lift(hardwareMap);

        //Run the following code while the Robot is in Init State
        while(opModeInInit()){

            if(gamepad1.dpad_down){
                extension.driveMotor(0.5);
            }
            else if(gamepad1.dpad_up){
                extension.driveMotor(-0.5);
            }
            else{
                extension.driveMotor(0);
            }

            telemetry.addData("Motor pos: ", extension.getPosition()[0]);
            telemetry.addData("Lift pos: ", lift.getPosition());
            telemetry.update();

        }





        while(opModeIsActive()){

//            if(gamepad1.right_bumper){
//                inputGripper.openGripper();
//
//            }
            if(gamepad1.start){
                lift.extendTo(710);
            }

            if(gamepad1.right_trigger > 0.8){
                lift.extendTo(800);
            }

            if(gamepad1.back){
                extension.reset();
                lift.reset();
                extension.extendTo((int)minPos);
                outputgripper.openGripper();
            }

            if(gamepad1.right_bumper){
                if(lift.getPosition() > 500 ) {
                    intakeWrist.goInside();
                }
            }
            else if(gamepad1.left_bumper){
                intakeWrist.goOutside();
            }


            if(gamepad1.left_stick_button){
                inputGripper.closeGripper();
            }
            else if(gamepad1.right_stick_button){
                inputGripper.openGripper();
            }




            if(gamepad1.a){
                outputWrist.goInside();
            }
            else if(gamepad1.b){
                outputWrist.goOutside();
            }

            if(gamepad1.x){
                outputgripper.closeGripper();
                sleep(200);
                inputGripper.openGripper();
                extension.extendTo((int)maxPos);
                while(extension.getPosition()[0] <= 0.4*maxPos){

                }
                inputGripper.closeGripper();
                intakeWrist.goOutside();

                sleep(1000);
                inputGripper.openGripper();
                outputWrist.goOutside();


                while(extension.getPosition()[0] <= maxPos-1){
                    if(gamepad1.dpad_left){
                        lift.driveMotor(1);
                    }
                    else if(gamepad1.dpad_right){
                        lift.driveMotor(-1);
                    }
                    else{
                        lift.driveMotor(0);
                    }

                    if(gamepad1.start){
                        lift.extendTo(710);
                    }

                    if(gamepad1.right_trigger > 0.8){
                        lift.extendTo(800);
                    }
                }
                sleep(2000);
                outputgripper.openGripper();

                sleep(500);
                outputWrist.goInside();
                goDown();
            }
            else if(gamepad1.y){
                outputgripper.openGripper();
            }

            if(gamepad1.dpad_left){
                lift.driveMotor(1);
            }
            else if(gamepad1.dpad_right){
                lift.driveMotor(-1);
            }
            else{
                lift.driveMotor(0);
            }

            if(gamepad1.dpad_down){
                goDown();

            }
            else if(gamepad1.dpad_up){


            }

            telemetry.addData("Motor pos: ", extension.getPosition()[0]);
            telemetry.addData("Lift pos: ", lift.getPosition());
            telemetry.update();


        }
    }

    void goDown(){
        extension.extendTo((int)minPos);
        outputgripper.openGripper();

    }
}
