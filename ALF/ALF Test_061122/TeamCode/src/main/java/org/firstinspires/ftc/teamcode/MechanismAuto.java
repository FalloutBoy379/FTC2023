package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.Input;
import org.firstinspires.ftc.teamcode.output.Output;

import java.util.concurrent.TimeUnit;


@TeleOp
public class MechanismAuto extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();
    Output.Wrist outputWrist = null;
    Input.Wrist intakeWrist = null;
    Input.Gripper inputGripper = null;
    Output.Gripper outputgripper = null;
    Output.Extension extension = null;
    Input.Lift lift = null;


    double targetExtensionPosition = 1600;
    double targetLiftPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        extension = new Output.Extension(hardwareMap);
        lift = new Input.Lift(hardwareMap);
        outputgripper = new Output.Gripper(hardwareMap);
        outputWrist = new Output.Wrist(hardwareMap);
        inputGripper = new Input.Gripper(hardwareMap);
        intakeWrist = new Input.Wrist(hardwareMap);

        time.reset();

        double[] extensionPosition = {0,0};
        double liftPosition = 0;

        telemetry.addData("Status: ", "Intialized");
        telemetry.update();

        while(opModeInInit()){
            telemetry.addData("Time in initialization: ", time.time(TimeUnit.MILLISECONDS));
            telemetry.update();
            if(gamepad1.dpad_down){
                extension.driveMotor(0.5);
            } else if(gamepad1.dpad_up){
                extension.driveMotor(-0.5);
            } else{
                extension.driveMotor(0);
            }
            if(gamepad1.dpad_right){
                lift.driveMotor(1);
            }
            else if(gamepad1.dpad_left){
                lift.driveMotor(-1);
            }
            else{
                lift.driveMotor(0);
            }
            extensionPosition = extension.getPosition();
            liftPosition = lift.getPosition();
            telemetry.addData("Extension Position(Left Motor): ", extensionPosition[0]);
            telemetry.addData("Extension Position(Right Motor): ", extensionPosition[1]);
            telemetry.addData("Lift position: ", liftPosition);
            telemetry.update();

        }

        lift.reset();
        extension.reset();



        while(opModeIsActive()){
            extensionPosition = extension.getPosition();
            liftPosition = lift.getPosition();

            if(gamepad1.dpad_up && gamepad1.right_bumper){
                extension.driveMotor(1);
            }
            else if(gamepad1.dpad_down && gamepad1.right_bumper){
                extension.driveMotor(-1);
            }
            else if(gamepad1.right_bumper){
                extension.driveMotor(0);
            }

            else if(gamepad1.x){
                extension.extendTo((int)targetExtensionPosition);
            }
            else if(gamepad1.y){
                extension.extendTo(0);
            }


            if(gamepad1.left_trigger > 0.8){
                targetExtensionPosition = extensionPosition[0];
            }



            if(gamepad1.right_trigger > 0.8){
                targetLiftPosition = liftPosition;
            }







            if(gamepad1.back){
                extension.reset();
            }


            if(Math.abs(extension.getTargetPos()[0] - extension.getPosition()[0])<=10){
                extension.driveMotor(0);
            }
            else{
                extension.extendTo((int)extension.getTargetPos()[0]);
            }
            telemetry.addLine("Extensions positions are: ");
            telemetry.addData("Left motor: ", extensionPosition[0]);
            telemetry.addData("Right motor: ", extensionPosition[1]);
            telemetry.addData("Lift position is: ", liftPosition);
            telemetry.addData("Left motor Current: ", extension.getCurrent()[0]);
            telemetry.addData("Right motor Current: ", extension.getCurrent()[1]);
            telemetry.update();
        }
    }

    void goDown(){
        extension.extendTo(0);
        outputgripper.openGripper();

    }

}
