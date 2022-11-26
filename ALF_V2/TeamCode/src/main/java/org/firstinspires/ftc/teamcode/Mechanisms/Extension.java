package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Extension {
    public enum POSITIONS{
        HIGH_JUNCTION,
        MED_JUNCTION,
        LOW_JUNCTION,
        GROUND_JUNCTION,
        TERMINAL,
        GRIPPING
    }
    //--------------------------------CONSTANTS----------------------------------
    private final double PULLEY_DIAMETER = 20.00; //mm
    private final double PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER * Math.PI;
    private final double EXTENSION_PER_REVOLUTION = PULLEY_CIRCUMFERENCE;
    private final double COUNTSPERREVOLUTION = 28.00;
    private final double EXTENSION_PER_COUNT = (1/COUNTSPERREVOLUTION)*EXTENSION_PER_REVOLUTION;
    private final double COUNT_PER_MM = 1/EXTENSION_PER_COUNT;
    private final double MAX_MOTOR_VEL_IDEAL = 2800;     //encoder counts per second



    //--------------------------------VARIABLES----------------------------------
    public static int targetMillimeters = 0;
    public static int[] POSITIONS = {0,0,0,0,0,0};

    //--------------------------------HARDWARE----------------------------------
    DcMotorEx leftMotor, rightMotor;
    TouchSensor homingSensor;

    public Extension(HardwareMap hardwareMap, Telemetry telemetry){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftExtension");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightExtension");
        homingSensor = hardwareMap.get(TouchSensor.class, "homingSensorExtension");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //TODO:Reverse any motor which rotates in the opposite direction
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        reset();

    }

    public void reset(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extendTo(double mm){
        int position = (int)(mm*COUNT_PER_MM);
        setTargetCount(position);

    }

    public void setTargetCount(int position){
        leftMotor.setTargetPosition(position);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(1);
        rightMotor.setTargetPosition(position);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(1);
    }

    public void driveMotors(double power){
        if(leftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(rightMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    
}
