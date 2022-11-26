package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intake.Input;
import org.firstinspires.ftc.teamcode.output.Output;


@TeleOp
public class inputAuto extends LinearOpMode {

    Input.Lift inputLift = null;
    Input.Wrist inputWrist = null;
    Input.Gripper inputGripper = null;
    Output.Extension outputExtension = null;
    Output.Gripper outputGripper = null;
    Output.Wrist outputWrist = null;

    final double CONE_DIFFERENCE = 30;   //mm
    double CONE1 = 43;
    double CONE2 = 0;
    double CONE3 = 0;
    double CONE4 = 0;
    double CONE5 = 0;


    final double OUTPUT_PULLEY_DIAMETER = 1; //mm
    final double GEAR_RATIO_OUTPUT = 9;
    final double COUNTS_PER_REV_OUTPUT = 28 * GEAR_RATIO_OUTPUT;
    final double OUTPUT_PULLEY_CIRCUMFERENCE = Math.PI * OUTPUT_PULLEY_DIAMETER;
    final double COUNTS_PER_MM_OUTPUT = COUNTS_PER_REV_OUTPUT/OUTPUT_PULLEY_CIRCUMFERENCE;

    int GRIPPING_POSITION = 0;
    int HIGH_POLE_POSITION = 0;
    int MEDIUM_POLE_POSITION = 0;
    int LOW_POLE_POSITION = 0;

    final double COUNTS_PER_REV_INPUT = 288;
    final double PULLEY_DIAMETER_INPUT = 20;    //mm
    final double PULLEY_CIRCUMFERENCE_INPUT = Math.PI * PULLEY_DIAMETER_INPUT;


    final double COUNTS_PER_MM_INPUT = COUNTS_PER_REV_INPUT / PULLEY_CIRCUMFERENCE_INPUT;

    int buttoncounter = 0;
    boolean[] lastFlag = new boolean[14];

    @Override
    public void runOpMode() throws InterruptedException {

        inputLift = new Input.Lift(hardwareMap);
        inputWrist = new Input.Wrist(hardwareMap);
        inputGripper = new Input.Gripper(hardwareMap);
        outputExtension = new Output.Extension(hardwareMap);
        outputWrist = new Output.Wrist(hardwareMap);
        outputGripper = new Output.Gripper(hardwareMap);

        inputLift.reset();

        outputGripper.closeGripper();
        outputWrist.goInside();
        inputLift.extendTo(800);
        inputGripper.closeGripper();
        outputGripper.openGripper();
        sleep(2000);
        inputWrist.goInside();
        inputLift.extendTo(650);
        CONE1 = 30;
        CONE2 = CONE1 + CONE_DIFFERENCE;
        CONE3 = CONE2 + CONE_DIFFERENCE;
        CONE4 = CONE3 + CONE_DIFFERENCE;
        CONE5 = CONE4 + CONE_DIFFERENCE;

        while(opModeInInit()){
            outputExtension.driveMotor(gamepad1.left_stick_y);
                if(gamepad1.back){
                    outputExtension.reset();
                }

                if(gamepad1.dpad_down){
                    outputGripper.closeGripper();
                }
                else{
                    outputGripper.openGripper();
                }
                if(gamepad1.a){
                    GRIPPING_POSITION = (int)outputExtension.getPosition()[0];
                }
                if(gamepad1.b){
                    LOW_POLE_POSITION = (int)outputExtension.getPosition()[0];
                }
                if(gamepad1.x){
                    MEDIUM_POLE_POSITION = (int)outputExtension.getPosition()[0];
                }
                if(gamepad1.y){
                    HIGH_POLE_POSITION = (int)outputExtension.getPosition()[0];
                }

                telemetry.addLine("Press A for setting GRIPPING POSITION");
                telemetry.addLine("Press B for setting LOW POLE POSITION");
                telemetry.addLine("Press X for setting MEDIUM POLE POSITION");
                telemetry.addLine("Press Y for setting HIGH POLE POSITION");
                telemetry.addData("Extension: ", outputExtension.getPosition()[0]);
                telemetry.addData("Lift: ", inputLift.getPosition());
                telemetry.update();
        }

        outputExtension.reset();



        while(opModeIsActive()){




            if(gamepad1.right_bumper && lastFlag[0] == false){
                outputExtension.extendTo(GRIPPING_POSITION);                      //Send output extension to be ready to accept new cone
                liftCounter();                                                    //Send input lift to required cone position
                sleep(3000);
                inputGripper.closeGripper();                                      //Grab the input gripper after 3 seconds
                outputGripper.openGripper();
                sleep(200);
                inputLift.extendTo(800);                                     //Send input lift to top position
                outputGripper.openGripper();                                      //Open the output gripper to be ready for cone
                sleep(1000);
                inputWrist.goInside();                                            //Rotate the wrist towards the output gripper
                inputLift.extendTo(650);
                sleep(1000);
                outputGripper.closeGripper();                                     //Close the output gripper and grab the conee
                sleep(400);
                inputGripper.openGripper();                                       //Open the input gripper after a small 100ms delay
                sleep(500);
                outputExtension.extendTo(HIGH_POLE_POSITION);

                while(Math.abs(outputExtension.getPosition()[0]) < Math.abs(HIGH_POLE_POSITION/2));
                outputWrist.goOutside();
                sleep(2000);
                outputGripper.openGripper();
                sleep(1000);
                outputGripper.closeGripper();
                outputWrist.goInside();
                lastFlag[0] = true;
            }

            if(gamepad1.left_bumper && lastFlag[1] == false){
                inputGripper.openGripper();
                sleep(500);
                inputGripper.closeGripper();
                inputWrist.goOutside();
                sleep(500);
                inputGripper.openGripper();
                lastFlag[1] = true;
            }

            if(gamepad1.a){

            }




            if(!gamepad1.a){
                lastFlag[2] = false;
            }
            if(!gamepad1.right_bumper){
                lastFlag[0] = false;
            }
            if(!gamepad1.left_bumper){
                lastFlag[1] = false;
            }


            telemetry.addData("Lift target pos", inputLift.getPosition());
            telemetry.addData("buttonCounter: ", buttoncounter);
            telemetry.update();
        }
    }

    int countFromMm_INPUT(double mm){
        return (int)(COUNTS_PER_MM_INPUT * mm);
    }

    double MmFromCount_INPUT(int count){
        return 1/ COUNTS_PER_MM_INPUT * count;
    }

    int countFromMm_OUTPUT(double mm){
        return (int)(COUNTS_PER_MM_OUTPUT * mm);
    }

    double MmFromCount_OUTPUT(double count){
        return 1/COUNTS_PER_MM_OUTPUT * count;
    }

    void liftCounter(){
        if(buttoncounter == 0){
            inputLift.extendTo(countFromMm_INPUT(CONE5));
            buttoncounter++;
        }
        else if(buttoncounter == 1){
            inputLift.extendTo(countFromMm_INPUT(CONE4));
            buttoncounter++;
        }
        else if(buttoncounter == 2){
            inputLift.extendTo(countFromMm_INPUT(CONE3));
            buttoncounter++;
        }
        else if(buttoncounter == 3){
            inputLift.extendTo(countFromMm_INPUT(CONE2));
            buttoncounter++;
        }
        else if(buttoncounter == 4){
            inputLift.extendTo(countFromMm_INPUT(CONE1));
            buttoncounter++;
        }
        if(buttoncounter > 4){
            buttoncounter = 0;
        }
    }




}
