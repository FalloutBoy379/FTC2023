package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.annotations.Configurable;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;

@TeleOp
@Config
public class FSM extends OpMode {

  //TODO: Initialization of Mechanisms
    //*******************************************INPUT*******************************************
    Intake input = null;
    private double current_gyro_error;
    public static double KP_Gyro = 0;
    public static double KI_Gyro = 0;
    public static double KD_Gyro = 0;
    private double p_gyro_error, i_gyro_error, d_gyro_error, prev_gyro_error, P_gyro, I_gyro, D_gyro;
    double targetAngle = 0;
    double comp = 0;

    boolean gripFlag = false;
    ElapsedTime gripTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
    public static int[] liftPosition = {125, 35, 65, 95, 125, 155};          //Possible to configure using Dashboard

    ElapsedTime inputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    INPUT_STATES inputState = INPUT_STATES.START;
    boolean xFlag = false;
    boolean yFlag = false;

    boolean FODrive = false;
    //*******************************************************************************************
    //*******************************************OUTPUT******************************************
    Extension output = null;
    public enum OUTPUT_STATES{
        START,
        PASS_CONE,
        DROP_HIGH,
        DROP_MED,
        STORAGE
    }

    String Storage = "Storage";
    String LOW_POLE = "LOW";
    String MED_POLE = "MED";
    String HIGH_POLE = "HIGH";
    String SELECTED_POLE = MED_POLE;

    int RECEIVE = 0,
    MID_JUNCTION = 1,
    HIGH_JUNCTION = 2;

    @Configurable
    public static int[] extensionPosition = {0
            , 78, 388};      //Possible to configure using Dashboard
//{35, 90, 400}
    public static int extensionPosition0 = extensionPosition[0];
    public static int extensionPosition1 = extensionPosition[1];
    public static int extensionPosition2 = extensionPosition[2];

    public int SAFETY_POSITION = 60;

    ElapsedTime outputTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    OUTPUT_STATES outputState = OUTPUT_STATES.START;
    //*******************************************************************************************

    //CONNECTIONS

    boolean isConeGrippedByOutput = false;
    boolean isConeGrippedByInput = false;

    boolean inFlag = false;

    SampleMecanumDrive drive;
    @Override
    public void init(){
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        input = new Intake(hardwareMap, telemetry);
        output = new Extension(hardwareMap, telemetry);
    }


    boolean zeroFlag = false;
    boolean zeroFlag2 = false;
    @Override
    public void init_loop() {

        if(input != null && output != null){
            telemetry.addData("Intake Sensor: ", Intake.homeSensor.isPressed());
            if(!Intake.homeSensor.isPressed() && !zeroFlag2){
                telemetry.addLine("Going to target");
                input.driveMotor(0.5);
            }
            else if(!zeroFlag2){
                zeroFlag2 = true;
                input.driveMotor(0);
                input.reset();
                input.extendLift(0);
            }
            telemetry.addData("Extension Sensor: ", Extension.homingSensor.isPressed());
            if(!Extension.homingSensor.isPressed() && !zeroFlag){
                telemetry.addLine("Going to target");
                output.driveMotors(0.5);
            }
            else if(!zeroFlag){
                zeroFlag = true;
                output.driveMotors(0);
                output.reset();
                output.extendTo(0);
            }
        }
    }

    @Override
    public void loop() {
        if(gamepad1.right_stick_button){
            FODrive = true;
        }
        else if(gamepad1.left_stick_button){
            FODrive = false;
        }
        if(Math.abs(gamepad1.right_stick_x) > 0.8 || gamepad1.dpad_up){
            inFlag = true;
        }
        else if(gamepad1.dpad_down && Math.abs(gamepad1.right_stick_x)<=0.8){
            inFlag = false;
        }
        extensionPosition[0] = extensionPosition0;
        extensionPosition[1] = extensionPosition1;
        extensionPosition[2] = extensionPosition2;


        if(!FODrive) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.5
                    )
            );
        }
        else{
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x * 0.5
                    )
            );
        }
//        driveRobot(-gamepad1.left_stick_y, -gamepad1.left_stick_x,-gamepad1.right_stick_x, gyro_read());
        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        if(gamepad1.right_bumper){
            SELECTED_POLE = HIGH_POLE;
        }
        else if(gamepad1.left_bumper){
            SELECTED_POLE = MED_POLE;
        }
        else if(gamepad1.left_trigger>0.8){
            SELECTED_POLE = Storage;
        }


        input.periodic();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();

        output.periodic();
        if((isConeGrippedByOutput && isConeGrippedByInput)&&(inputState != INPUT_STATES.PASS_CONE && outputState != OUTPUT_STATES.PASS_CONE)){
            telemetry.addLine("RULE BOOK VIOLATION: TWO CONES HELD TOGETHER!");
//            requestOpModeStop();
        }

        //FSM For Input lift
        switch (inputState){
            case START:                                //If the Input State is START
                telemetry.addLine("INPUT STATE: START");
                input.extendLift(liftPosition[GENERAL_CONE]);              //Rotate the motor to 0 mm
                telemetry.addData("Lift Position:", input.getLiftPosition());
                telemetry.addData("Home Status Input:", input.isHomed());
//                if(gamepad1.dpad_left){
//                    input.openInputGripper();
//                }
//                else if(gamepad1.dpad_right){
//                    input.closeInputGripper();
//                }
                if(Math.abs(input.getLiftPosition())-Math.abs(liftPosition[GENERAL_CONE]) < 3  && !inFlag){     //Wait for the lift to come down and the motor to spin
                    input.openInputGripper();          //Open gripper to get ready for the new cone
                    input.goOutside();
                    gripFlag = false;
                    gripTimer.reset();
                }
                else{
                    if(!gripFlag) {
                        input.closeInputGripper();             //Close the Input Gripper for safety of parts
                        sleep(100);
                        gripFlag = true;
                        input.wristIdle();                     //Idle position for wrist
                    }
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
                    telemetry.addData("safetyCheck:", safetyCheckBeforeTransferForInput());
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
                 if(isConeGrippedByOutput) {
                     if (inputTimer.milliseconds() > 300 && isConeGrippedByInput){
                         input.openInputGripper();                   //If Output has gripped cone, release the cone from input
                     isConeGrippedByInput = false;
                 }
                 }
                 else{
                     inputTimer.reset();
                 }
                }
                break;
        }





        switch(outputState){
            case START:
                telemetry.addLine("OUTPUT STATE: START");
                output.extendTo(extensionPosition[RECEIVE]);            //TODO: CHANGE THIS FOR FINAL CODE

                if(gamepad1.x || xFlag){
                    output.extendTo(extensionPosition[RECEIVE]);
                    if(Math.abs(output.getExtensionPosition()[0])-Math.abs(extensionPosition[RECEIVE])< SAFETY_POSITION){
                        output.openGripper();
                    }
                    if(Math.abs(output.getExtensionPosition()[0])-Math.abs(extensionPosition[RECEIVE]) < 10){
                        outputState = OUTPUT_STATES.PASS_CONE;
                        outputTimer.reset();
                    }
                }
                else{
                    output.goInside();
                    output.closeGripper();
                }

                break;

            case PASS_CONE:
                telemetry.addLine("OUTPUT STATE: PASS_CONE");
                output.extendTo(extensionPosition[RECEIVE]);
                if(inputState == INPUT_STATES.PASS_CONE && outputTimer.milliseconds()>1000){
                    output.closeGripper();
                    isConeGrippedByOutput = true;
                    if(!isConeGrippedByInput){
                        if(Objects.equals(SELECTED_POLE, HIGH_POLE)) {
                            output.extendTo(extensionPosition[HIGH_JUNCTION]);
                            if(Math.abs(output.getExtensionPosition()[0]) >= Math.abs(extensionPosition[HIGH_JUNCTION]/2.0)){
                                input.closeInputGripper();
                                input.wristIdle();
                                output.goOutside();
                                outputState = OUTPUT_STATES.DROP_HIGH;
                                outputTimer.reset();
                            }

                        }
                        else if(Objects.equals(SELECTED_POLE, MED_POLE)){
                            output.extendTo(extensionPosition[MID_JUNCTION]);
                            if(Math.abs(output.getExtensionPosition()[0]) >= Math.abs(extensionPosition[MID_JUNCTION]/2.0)){
                                input.closeInputGripper();
                                input.wristIdle();
                                output.goOutside();
                                outputState = OUTPUT_STATES.DROP_MED;
                                outputTimer.reset();
                            }
                        }
                        else if(Objects.equals(SELECTED_POLE, Storage)){
                            output.extendTo(extensionPosition[RECEIVE]);
                            outputState = OUTPUT_STATES.DROP_MED;
                            outputTimer.reset();
                        }
                    }
                }
                break;

            case DROP_MED:
                telemetry.addLine("OUTPUT STATE: DROP_MED");
                checkOutputState();
                if(gamepad1.y && !yFlag){
                    yFlag = true;
                    output.openGripper();
                    isConeGrippedByOutput = false;
                    xFlag = false;
                    outputTimer.reset();
                }
                if(outputTimer.milliseconds() > 500 && yFlag){
                    yFlag = false;
                    inputState = INPUT_STATES.START;
                    outputState = OUTPUT_STATES.START;
                }
                break;

            case DROP_HIGH:
                checkOutputState();
                telemetry.addLine("OUTPUT STATE: DROP_HIGH");
                if(gamepad1.y && !yFlag){
                    yFlag = true;
                    output.openGripper();
                    isConeGrippedByOutput = false;
                    xFlag = false;
                    outputTimer.reset();
                }
                if(outputTimer.milliseconds() > 500 && yFlag){
                    yFlag = false;
                    inputState = INPUT_STATES.START;
                    outputState = OUTPUT_STATES.START;
                }
                break;

            case STORAGE:
              checkOutputState();
              telemetry.addLine("OUTPUT STATE: STORAGE");
                if(gamepad1.y){
//                    yFlag = true;
                    output.goOutside();
//                    xFlag = false;
                    outputTimer.reset();
                }


        }
    }


    public void initInput(){

    }


    public boolean safetyCheckBeforeTransferForOutput(){


        return false;
    }

    public void checkOutputState(){
        if(Objects.equals(SELECTED_POLE, HIGH_POLE)) {
            output.extendTo(extensionPosition[HIGH_JUNCTION]);
            outputState = OUTPUT_STATES.DROP_HIGH;
        }
        else if(Objects.equals(SELECTED_POLE, MED_POLE)){
            output.extendTo(extensionPosition[MID_JUNCTION]);
            outputState = OUTPUT_STATES.DROP_MED;
        }
        else if(Objects.equals(SELECTED_POLE, Storage)){
            output.extendTo(extensionPosition[RECEIVE]);
            outputState = OUTPUT_STATES.STORAGE;
        }
    }

    public void driveRobot(double x, double y, double w, double theta){
        if(!FODrive) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            x,
                            y,
                            w
                    )
            );
        }
        else{
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    x,
                    y
            ).rotated(-theta);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            w
                    )
            );
        }
    }

    public double gyro_read() {
        return -AngleUnit.normalizeDegrees(Math.toDegrees(drive.getPoseEstimate().getHeading()));
    }

    public double gyroPID(double target, double current){
        current_gyro_error = target-current;
//        KP_Gyro = 3.0;
//        KI_Gyro = 0;
//        KD_Gyro = 70.0;

        p_gyro_error = current_gyro_error;
        i_gyro_error = prev_gyro_error + current_gyro_error;
        d_gyro_error = current_gyro_error - prev_gyro_error;

        P_gyro = KP_Gyro * p_gyro_error;
        I_gyro = KI_Gyro * i_gyro_error;
        D_gyro = KD_Gyro * d_gyro_error;

        double corr = -(P_gyro + D_gyro + I_gyro);
        prev_gyro_error = current_gyro_error;
        return corr;
    }

    public boolean safetyCheckBeforeTransferForInput(){
        if((Math.abs(output.getExtensionPosition()[0]) - liftPosition[RECEIVE]) < 100){
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
