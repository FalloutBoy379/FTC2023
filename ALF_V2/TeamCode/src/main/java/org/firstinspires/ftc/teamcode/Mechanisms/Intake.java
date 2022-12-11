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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
public class Intake extends SubsystemBase {
    Telemetry localTelemetry;

    public static enum POSITIONS{
        CONE5, CONE4, CONE3, CONE2, CONE1
    }

    //--------------------------------CONSTANTS----------------------------------
    private static final double INPUT_PULLEY_DIAMETER = 30; //mm
    private static final double INPUT_GEAR_RATIO = 8.4;
    private static final double INPUT_PULLEY_CIRCUMFERENCE = INPUT_PULLEY_DIAMETER * Math.PI;
    private static final double INPUT_EXTENSION_PER_REVOLUTION = INPUT_PULLEY_CIRCUMFERENCE;
    private static final double INPUT_COUNTSPERREVOLUTION = 28.00 * INPUT_GEAR_RATIO;
    private static final double INPUT_EXTENSION_PER_COUNT = INPUT_EXTENSION_PER_REVOLUTION/INPUT_COUNTSPERREVOLUTION;
    private static final double INPUT_COUNT_PER_MM = 1.0/(INPUT_EXTENSION_PER_COUNT);
    private static final double INPUT_MAX_MOTOR_VEL_IDEAL = 6000/INPUT_GEAR_RATIO;     //encoder counts per second

    private static final double INPUT_MAX_EXTENSION = 600; //mm




    static double INPUT_IN = 0.1;
    static double INPUT_OUT = 0.9;
    static double INPUT_GRIPPER_CLOSE = 1;
    static double INPUT_GRIPPER_OPEN = 0.5;


    public static int targetMillimeters_input = 0;
    public static int MAX_MM = 250;
    public static int MIN_MM = 0;

    public static DcMotorEx liftMotor;
    public static Servo Wrist, Claw;
    public static TouchSensor homeSensor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        localTelemetry = telemetry;
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        Wrist = hardwareMap.get(Servo.class, "intakeWrist");
        Claw = hardwareMap.get(Servo.class, "intakeClaw");
        homeSensor = hardwareMap.get(TouchSensor.class, "homeSensorInput");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO:Reverse any motor which rotates in the opposite direction
//        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);




        telemetry.addLine("Initialized Intake successfully!");
        reset();
    }

    public void disableMotors(){
        liftMotor.setMotorDisable();
    }

    @Override
    public void periodic() {
        localTelemetry.addData("COUNTPERMM", INPUT_COUNT_PER_MM);
        localTelemetry.addData("Sensor value: ", homeSensor.isPressed());
        localTelemetry.addData("Left: ", liftMotor.getCurrentPosition());
        localTelemetry.addData("Motor: ", liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }

    public void extendLift(double mm){
//        mm = Range.clip(mm, 0, INPUT_MAX_EXTENSION);
        int position = (int)(mm*INPUT_COUNT_PER_MM);
        setLiftTargetCount(-position);

    }

    public void setLiftTargetCount(int position){
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
    }


    public void driveMotor(double power){
        if(liftMotor.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        liftMotor.setPower(power);
    }

    public double getPosition(Servo positonMotor){
        return positonMotor.getPosition();
    }

    public double getWristPosition(){
        return getPosition(Wrist);
    }

    public double getGripperPosition(){
            return getPosition(Claw);
    }

    public double getLiftPosition(){
        return liftMotor.getCurrentPosition()* INPUT_EXTENSION_PER_COUNT;
    }

    public double getLiftCurrent(){
        return liftMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public void reset(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void closeInputGripper(){
        Claw.setPosition(INPUT_GRIPPER_CLOSE);
    }

    public void openInputGripper(){
        Claw.setPosition(INPUT_GRIPPER_OPEN);
    }

    public void goOutside(){
        Wrist.setPosition(INPUT_OUT);
    }

    public void goInside(){
        Wrist.setPosition(INPUT_IN);
    }

    public void wristIdle(){Wrist.setPosition((INPUT_IN+INPUT_OUT)/2);}

    public boolean isHomed(){
        return homeSensor.isPressed();}
}
