package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Extension extends SubsystemBase {
    Telemetry localTelemetry;
    public static enum POSITIONS{
        HIGH_JUNCTION,
        MED_JUNCTION,
        LOW_JUNCTION,
        GROUND_JUNCTION,
        TERMINAL,
        GRIPPING
    }
    //--------------------------------CONSTANTS----------------------------------
    private static final double PULLEY_DIAMETER = 24.16; //mm
    private static final double GEAR_RATIO = 8.4;
    private static final double PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER * Math.PI;
    private static final double EXTENSION_PER_REVOLUTION = PULLEY_CIRCUMFERENCE;
    private static final double COUNTSPERREVOLUTION = 28.00 * GEAR_RATIO;
    private static final double EXTENSION_PER_COUNT = EXTENSION_PER_REVOLUTION/COUNTSPERREVOLUTION;
    private static final double COUNT_PER_MM = 1.0/(EXTENSION_PER_COUNT*1.010344827586207);
    private static final double MAX_MOTOR_VEL_IDEAL = 6000/GEAR_RATIO;     //encoder counts per second

    private static final double MAX_EXTENSION = 600; //mm

  /*
            ___
           /
      h2  /
         /
    d1  /
       /
    h1/
     /
  GC |
   */

    private static final double GROUND_CLEARANCE = 86.75; //mm
    private static final double h1 = 425;  //mm
    private static final double h2 = 0;    //mm
    private static final double l1 = 0;    //mm
    private static final double theta = 0; //mm

    static double OUTPUT_IN = 0;
    static double OUTPUT_OUT = 1;
    static double GRIPPER_CLOSE = 1;
    static double GRIPPER_OPEN = 0;


    //--------------------------------VARIABLES----------------------------------
    public static int targetMillimeters = 0;
    public static int[] POSITIONS = {0,0,0,0,0,0};

    //--------------------------------HARDWARE----------------------------------
    public static DcMotorEx leftMotor, rightMotor;
    public static TouchSensor homingSensor;
    public static Servo leftServo, rightServo, gripper;

    public Extension(HardwareMap hardwareMap, Telemetry telemetry){
        localTelemetry = telemetry;
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftExtension");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightExtension");
        homingSensor = hardwareMap.get(TouchSensor.class, "homingSensorExtension");
        leftServo = hardwareMap.get(Servo.class, "leftExtension");
        rightServo = hardwareMap.get(Servo.class, "rightExtension");
        gripper = hardwareMap.get(Servo.class, "outputGripper");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //TODO:Reverse any motor which rotates in the opposite direction
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


//        telemetry.addLine("");



        telemetry.addLine("Initialized Extension successfully!");
        reset();
        extendTo(0);

    }

    public void disableMotors(){
        leftMotor.setMotorDisable();
        rightMotor.setMotorDisable();
    }

    @Override
    public void periodic() {
        localTelemetry.addData("COUNTPERMM", COUNT_PER_MM);
        localTelemetry.addData("Sensor value: ", homingSensor.isPressed());
        localTelemetry.addData("Left: ", leftMotor.getCurrentPosition());
        localTelemetry.addData("Right: ", rightMotor.getCurrentPosition());
    }

    public double[] getExtensionPosition(){
        return new double[]{leftMotor.getCurrentPosition()*EXTENSION_PER_COUNT, rightMotor.getCurrentPosition()*EXTENSION_PER_COUNT};
    }

    public double getWristPosition(){
        return leftServo.getPosition();
    }

    public double getGripperPosition(){
        return gripper.getPosition();
    }

    public boolean isGripperOpen(){
        if(getGripperPosition() == GRIPPER_OPEN){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isWristInside(){
        if(getWristPosition() == OUTPUT_IN){
            return true;
        }
        else{
            return false;
        }
    }

    public void closeGripper(){
        gripper.setPosition(GRIPPER_CLOSE);
    }

    public void openGripper(){
        gripper.setPosition(GRIPPER_OPEN);
    }

    public void goOutside(){
        leftServo.setPosition(OUTPUT_OUT);
        rightServo.setPosition(1-OUTPUT_OUT);
    }

    public void goInside(){
        leftServo.setPosition(OUTPUT_IN);
        rightServo.setPosition(1-OUTPUT_IN);
    }

    public void reset(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extendTo(double mm){
        mm = Range.clip(mm, 0, MAX_EXTENSION);
        int position = (int)(mm*COUNT_PER_MM);
        setTargetCount(-position);

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
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    
}
