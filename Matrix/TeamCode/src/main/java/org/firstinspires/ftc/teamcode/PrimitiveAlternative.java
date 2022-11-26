package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class PrimitiveAlternative extends LinearOpMode {

    public final int GRIPPING_POSITION = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;

    boolean[] flags = {false, false, false, false};
    boolean[] flags2 = {false, false};
    public final double COUNTS_PER_DEGREE = 5.423728813559322;
    public final double COUNTS_PER_MM = 4.7558;
    public final int[] POSITIONS = {510, -1040, -1980, -2850};

    public final double SLIDER_IN_POS = 1;
    public final double SLIDER_OUT_POS = 0;

    public final double WRIST_TOP_POS = 0.59;
    public final double WRIST_INTAKE_POS = 0.5;
    public final double WRIST_DEFAULT_POS = 0;

    public final double GRIPPER_CLOSE_POSITION = 0;
    public final double GRIPPER_INTAKE_POSITION = 1;
    public final double GRIPPER_OPEN_POSITION = 0.5;

    public static double Left_Motor_P;
    public static double Left_Motor_I;
    public static double Left_Motor_D;

    public static double Right_Motor_P;
    public static double Right_Motor_I;
    public static double Right_Motor_D;

    DcMotorEx elevatorLeft = null;
    DcMotorEx elevatorRight = null;
    DcMotorEx turret = null;
    Servo slider;

    @Override
    public void runOpMode() throws InterruptedException {
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        slider = hardwareMap.get(Servo.class, "slider");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(36, -60, 0));
        double position = 0;
        while(opModeInInit()){
            if(gamepad1.a){
                position++;
            }
            else if(gamepad1.b){
                position--;
            }

            if(gamepad1.x){
                gripper.setPosition(GRIPPER_CLOSE_POSITION);
            }
            else if(gamepad1.y){
                gripper.setPosition(GRIPPER_OPEN_POSITION);
            }
            telemetry.addData("Position in mm:", elevatorLeft.getCurrentPosition() * (1/COUNTS_PER_MM));
            telemetry.addData("Position in counts:", elevatorLeft.getCurrentPosition());
            telemetry.update();
            extendToMM(position);
        }
        wrist.setPosition(WRIST_DEFAULT_POS);
        slider.setPosition(SLIDER_IN_POS);


        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        position = 0;

        double degrees = 0;
        Left_Motor_P = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;
        Left_Motor_I = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i;
        Left_Motor_D = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d;

        Right_Motor_P = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;
        Right_Motor_I = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i;
        Right_Motor_D = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d;

        PIDFCoefficients leftMotorCoeff = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients rightMotorCoeff = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);


        while(opModeIsActive()){

            drive.update();

            leftMotorCoeff.p = 10;
            leftMotorCoeff.d = 3;

            rightMotorCoeff.p = 10;
            rightMotorCoeff.d = 3;

            elevatorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, leftMotorCoeff);
            elevatorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, rightMotorCoeff);



            Pose2d robotPose = drive.getPoseEstimate();

            Vector2d input = new Vector2d(-gamepad1.left_stick_y * 1, -gamepad1.left_stick_x * 1).rotated(-robotPose.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * 1));

            telemetry.addData("X", robotPose.getX());
            telemetry.addData("Y", robotPose.getY());
            telemetry.addData("Heading", robotPose.getHeading());
            if(gamepad1.dpad_right){
                extendTo(POSITIONS[GRIPPING_POSITION]);
                wrist.setPosition(WRIST_INTAKE_POS);
            }
            else if(gamepad1.dpad_down){
                extendTo(POSITIONS[LOW_POLE]);
                slider.setPosition(SLIDER_IN_POS);
                wrist.setPosition(WRIST_INTAKE_POS);
            }
            else if(gamepad1.dpad_left){
                extendTo(POSITIONS[MID_POLE]);
                wrist.setPosition(WRIST_INTAKE_POS);
            }
            else if(gamepad1.dpad_up){
                extendTo(POSITIONS[HIGH_POLE]);
            }

            if(elevatorLeft.getCurrentPosition() >  2800){

                wrist.setPosition(WRIST_TOP_POS);
            }


            if(gamepad1.right_bumper && gripper.getPosition() != GRIPPER_OPEN_POSITION && (flags[2] == false)){
                gripper.setPosition(GRIPPER_OPEN_POSITION);
                flags[2] = true;
            }
            else if(gamepad1.right_bumper && gripper.getPosition() == GRIPPER_OPEN_POSITION && (flags[2] == false)){
                gripper.setPosition(GRIPPER_CLOSE_POSITION);
                flags[2] = true;
            }

            if(!gamepad1.right_bumper){
                flags[2] = false;
            }


            if(gamepad1.left_bumper && (flags[3]==false) && (flags2[0] == false)){
                wrist.setPosition(WRIST_INTAKE_POS);
                flags[3] = true;
                flags2[0] = true;
            }
            else if(gamepad1.left_bumper && (flags[3]==false) && (flags2[0] == true)){
                wrist.setPosition(WRIST_TOP_POS);
                flags[3] = true;
                flags2[0] = false;
            }

            if(!gamepad1.left_bumper){
                flags[3] = false;
            }

            if(gamepad1.x && !flags[0]){
                degrees = degrees - 90;
                flags[0] = true;
            }
            if(gamepad1.b && !flags[1]){
                degrees = degrees + 90;
                flags[1] = true;
            }

            if(!gamepad1.b){
                flags[1] = false;
            }
            if(!gamepad1.x){
                flags[0] = false;
            }

            setTurretDegree(degrees);

            slider.setPosition(-gamepad1.left_trigger + gamepad1.right_trigger);

//            extendTo((int)position);

            telemetry.addData("Position Left", elevatorLeft.getCurrentPosition());
            telemetry.addData("Position Left", elevatorRight.getCurrentPosition());
            telemetry.addData("Current Left", elevatorLeft.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Current Right", elevatorRight.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }

    void extendTo(int pos){
        elevatorLeft.setTargetPosition(pos);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(1);
        elevatorRight.setTargetPosition(pos);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
    }

    void extendToMM(double mm){
        int counts = (int)(COUNTS_PER_MM*mm);
        extendTo(counts);
    }

    public double map(double x, double in_min, double in_max, double out_min, double out_max){
        return ((x-in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min);
    }

    public void sliderMM(double pos){
        pos = map(pos, 0, 11, 0, 1);
        slider.setPosition(pos);
    }

    void setTurretDegree(double degree){
        turret.setTargetPosition((int)(COUNTS_PER_DEGREE*degree));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }
}
