package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.annotations.Configurable;

import java.util.Objects;

@TeleOp
@Config
public class FSM extends OpMode {

  //TODO: Initialization of Mechanisms
    //*******************************************INPUT*******************************************
    Intake input = null;
    public enum INPUT_STATES{
        START,
        GET_CONE,
        PASS_CONE
    }


    int
        TRANSFER = 0,
        GENERAL_CONE = 1,
        CONE2 = 2,
        CONE3 = 3,
        CONE4 = 4,
        CONE5 = 5;

    @Configurable
    public static int[] liftPosition = {140, 30, 0, 0, 0, 0};          //Possible to configure using Dashboard

    ElapsedTime inputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    INPUT_STATES inputState = INPUT_STATES.START;
    boolean xFlag = false;
    boolean yFlag = false;
    //*******************************************************************************************
    //*******************************************OUTPUT******************************************
    Extension output = null;
    public enum OUTPUT_STATES{
        START,
        PASS_CONE,
        DROP_HIGH,
        DROP_MED
    }

    String LOW_POLE = "LOW";
    String MED_POLE = "MED";
    String HIGH_POLE = "HIGH";
    String SELECTED_POLE = MED_POLE;

    int RECEIVE = 0,
    MID_JUNCTION = 1,
    HIGH_JUNCTION = 2;

    @Configurable
    public static int[] extensionPosition = {35, 35, 540};      //Possible to configure using Dashboard

    public int SAFETY_POSITION = 60;

    ElapsedTime outputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    OUTPUT_STATES outputState = OUTPUT_STATES.START;
    //*******************************************************************************************

    //CONNECTIONS

    boolean isConeGrippedByOutput = false;
    boolean isConeGrippedByInput = false;

    @Override
    public void init(){
        input = new Intake(hardwareMap, telemetry);
        output = new Extension(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        if(input != null && output != null){

        }
    }

    @Override
    public void loop() {

        if((isConeGrippedByOutput && isConeGrippedByInput)&&(inputState != INPUT_STATES.PASS_CONE && outputState != OUTPUT_STATES.PASS_CONE)){
            telemetry.addLine("<p style=\"color:red\">RULE BOOK VIOLATION: TWO CONES HELD TOGETHER!</p>");
            requestOpModeStop();
        }

        //FSM For Input lift
        switch (inputState){
            case START:                                //If the Input State is START
                telemetry.addLine("INPUT STATE: START");
                input.wristIdle();                     //Idle position for wrist
                input.closeInputGripper();             //Close the Input Gripper for safety of parts
                input.extendLift(liftPosition[GENERAL_CONE]);              //Rotate the motor to 0 mm
                if(Math.abs(input.getLiftPosition()) < 10 && input.isHomed()){     //Wait for the lift to come down and the motor to spin
                    input.openInputGripper();          //Open gripper to get ready for the new cone
                }
                inputTimer.reset();                    //Keep resetting the timer in START state
                if(gamepad1.x && !xFlag && !isConeGrippedByOutput && !isConeGrippedByInput){   //Check for the X button and whether cone is held already
                    input.closeInputGripper();         //Grab the cone immediately
                    xFlag = true;
                    isConeGrippedByInput = true; //TODO: set xFlag to false somewhere
                }
                if(xFlag){
                    inputState = INPUT_STATES.GET_CONE;
                }

                break;
            case GET_CONE:                             //The cone is being grabbed will be now waiting to be transferred
                telemetry.addLine("INPUT STATE: GET_CONE");
                if(inputTimer.milliseconds() > 200){   //Give a buffer of 200 ms for the cone to be grabbed properly
                    input.extendLift(liftPosition[TRANSFER]);    //Extend lift to the transfer position
                    if(safetyCheckBeforeTransferForInput()){             //Check if it is safe to rotate the wrist
                        input.goInside();                        //Rotate Cone towards the inside
                        inputState = INPUT_STATES.PASS_CONE;
                        inputTimer.reset();
                    }
                }
                break;
            case PASS_CONE:                           //Cone is ready to be passed into the output
                telemetry.addLine("INPUT STATE: PASS_CONE");
                if(outputState == OUTPUT_STATES.PASS_CONE){      //Proceed only when output is ready for passing
                 if(isConeGrippedByOutput){
                     input.openInputGripper();                   //If Output has gripped cone, release the cone from input
                     isConeGrippedByInput = false;
                     if(!output.isWristInside()){
                         inputState = INPUT_STATES.START;
                         xFlag = false;
                     }
                 }
                }
                break;
        }





        switch(outputState){
            case START:
                output.extendTo(liftPosition[RECEIVE]);            //TODO: CHANGE THIS FOR FINAL CODE
                output.goInside();
                output.closeGripper();
                if(gamepad1.x || xFlag){
                    output.extendTo(liftPosition[RECEIVE]);
                    if(Math.abs(output.getExtensionPosition()[0]-liftPosition[RECEIVE]) < SAFETY_POSITION){
                        output.openGripper();
                    }
                    if(Math.abs(output.getExtensionPosition()[0]-liftPosition[RECEIVE]) < 10){
                        outputState = OUTPUT_STATES.PASS_CONE;
                    }
                }

                break;

            case PASS_CONE:
                if(inputState == INPUT_STATES.PASS_CONE){
                    output.closeGripper();
                    isConeGrippedByOutput = true;
                    if(!isConeGrippedByInput){
                        if(Objects.equals(SELECTED_POLE, HIGH_POLE)) {
                            output.extendTo(liftPosition[HIGH_JUNCTION]);
                            if(output.getExtensionPosition()[0] >= liftPosition[HIGH_JUNCTION]/2.0){
                                output.goOutside();
                                outputState = OUTPUT_STATES.DROP_HIGH;
                                outputTimer.reset();
                            }

                        }
                        else if(Objects.equals(SELECTED_POLE, MED_POLE)){
                            output.extendTo(liftPosition[MID_JUNCTION]);
                            if(output.getExtensionPosition()[0] >= liftPosition[MID_JUNCTION]/2.0){
                                output.goOutside();
                                outputState = OUTPUT_STATES.DROP_MED;
                                outputTimer.reset();
                            }
                        }
                    }
                }
                break;

            case DROP_MED:
                if(gamepad1.y && !yFlag){
                    yFlag = true;
                    output.openGripper();
                    isConeGrippedByOutput = false;
                    outputTimer.reset();
                }
                if(outputTimer.milliseconds() > 1000 && yFlag){
                    yFlag = false;
                    outputState = OUTPUT_STATES.PASS_CONE;
                }
                break;

            case DROP_HIGH:
                if(gamepad1.y && !yFlag){
                    yFlag = true;
                    output.openGripper();
                    isConeGrippedByOutput = false;
                    outputTimer.reset();
                }
                if(outputTimer.milliseconds() > 500 && yFlag){
                    yFlag = false;
                    outputState = OUTPUT_STATES.START;
                }
                break;


        }
    }


    public void initInput(){

    }


    public boolean safetyCheckBeforeTransferForOutput(){


        return false;
    }

    public boolean safetyCheckBeforeTransferForInput(){
        if((Math.abs(output.getExtensionPosition()[0]) - liftPosition[RECEIVE]) < 60){
            if(output.isWristInside()){
                return output.isGripperOpen();
            }
            else{
                return true;
            }
        }
        else{
            return true;
        }
    }
}
