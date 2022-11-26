package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.Vision.AprilTagDetection.*;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a TILE_LENGTH-by-TILE_LENGTH square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "drive", name="AutoBlueRightTest")
public class autoTest extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.04;

    int ID_TAG_OF_INTEREST_1 = 3; // Tag ID 10 from the 36h11 family
    int ID_TAG_OF_INTEREST_2 = 7;
    int ID_TAG_OF_INTEREST_3 = 9;
    AprilTagDetection tagOfInterest = null;





    public final static double TILE_LENGTH = 24; // in
    public final int GRIPPING_POSITION = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;

    boolean[] flags = {false, false, false};
    public final double COUNTS_PER_DEGREE = 5.423728813559322;
    public final double COUNTS_PER_MM = 4.7558;
    public final int[] POSITIONS = {520, -9040, -1970, -2800};

    public final double[] CONE_HEIGHTS = {555, 417.326, 279.652, 136.978, -5.696};

    public final double SLIDER_IN_POS = 1;
    public final double SLIDER_OUT_POS = 0;

    public final double WRIST_TOP_POS = 0.59;
    public final double WRIST_INTAKE_POS = 0.5;
    public final double WRIST_DEFAULT_POS = 0;

    public final double GRIPPER_CLOSE_POSITION = 0;
    public final double GRIPPER_INTAKE_POSITION = 1;
    public final double GRIPPER_OPEN_POSITION = 0.5;


    public final Vector2d PICKING_POSITION = new Vector2d(-52.5, 14);
//    public static double Left_Motor_P;
//    public static double Left_Motor_I;
//    public static double Left_Motor_D;
//
//    public static double Right_Motor_P;
//    public static double Right_Motor_I;
//    public static double Right_Motor_D;


    DcMotorEx elevatorLeft = null;
    DcMotorEx elevatorRight = null;
    DcMotorEx turret = null;
    Servo slider;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        slider = hardwareMap.get(Servo.class, "slider");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        Left_Motor_P = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;
//        Left_Motor_I = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i;
//        Left_Motor_D = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d;
//
//        Right_Motor_P = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p;
//        Right_Motor_I = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i;
//        Right_Motor_D = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d;




        Pose2d startPose = new Pose2d(-36, 63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);



        TrajectorySequence startToHighPole = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->extendTo(POSITIONS[HIGH_POLE]))
                .addTemporalMarker(()->setTurretDegree(-131))
                .lineToSplineHeading(new Pose2d(-36,12, Math.toRadians(-180)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL * 0.3, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->wrist.setPosition(WRIST_TOP_POS))
                .addTemporalMarker(()->{slider.setPosition(0.47);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(0.05)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .waitSeconds(0.8)
                .build();


        TrajectorySequence goToPickPos = drive.trajectorySequenceBuilder(startToHighPole.end())
                .addTemporalMarker(()->wrist.setPosition(WRIST_INTAKE_POS))
                .addTemporalMarker(()->{setTurretDegree(1);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->slider.setPosition(SLIDER_OUT_POS))
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .lineToConstantHeading(new Vector2d(-52.875, 13.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->extendTo((int)CONE_HEIGHTS[4]))
                .waitSeconds(0.5)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
                .waitSeconds(0.6)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(0.2)
                .addTemporalMarker(()->slider.setPosition(0.5))
                .waitSeconds(0.3)
                .build();

        TrajectorySequence drop = drive.trajectorySequenceBuilder(goToPickPos.end())
                .addTemporalMarker(()->extendTo(POSITIONS[HIGH_POLE]))
                .addTemporalMarker(()->setTurretDegree(-132))
                .lineToConstantHeading(new Vector2d(-36, 12))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->wrist.setPosition(WRIST_TOP_POS))
                .addTemporalMarker(()->{slider.setPosition(0.47);})
                .waitSeconds(1)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(1)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence drop1 = drive.trajectorySequenceBuilder(goToPickPos.end())
                .addTemporalMarker(()->extendTo(POSITIONS[HIGH_POLE]))
                .addTemporalMarker(()->setTurretDegree(-133))
                .lineToConstantHeading(new Vector2d(-36, 11.8))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->wrist.setPosition(WRIST_TOP_POS))
                .addTemporalMarker(()->{slider.setPosition(0.47);})
                .waitSeconds(1)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(1)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence drop2 = drive.trajectorySequenceBuilder(goToPickPos.end())
                .addTemporalMarker(()->extendTo(POSITIONS[HIGH_POLE]))
                .addTemporalMarker(()->setTurretDegree(-134))
                .lineToConstantHeading(new Vector2d(-36, 11.8))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->wrist.setPosition(WRIST_TOP_POS))
                .addTemporalMarker(()->{slider.setPosition(0.47);})
                .waitSeconds(1)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(1)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence drop3 = drive.trajectorySequenceBuilder(goToPickPos.end())
                .addTemporalMarker(()->extendTo(POSITIONS[HIGH_POLE]))
                .addTemporalMarker(()->setTurretDegree(-135))
                .lineToConstantHeading(new Vector2d(-36.2, 11.6))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->wrist.setPosition(WRIST_TOP_POS))
                .addTemporalMarker(()->{slider.setPosition(0.47);})
                .waitSeconds(1)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(1)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence drop4 = drive.trajectorySequenceBuilder(goToPickPos.end())
                .addTemporalMarker(()->extendTo(POSITIONS[HIGH_POLE]))
                .addTemporalMarker(()->setTurretDegree(-136))
                .lineToConstantHeading(new Vector2d(-35.9, 11.4))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->wrist.setPosition(WRIST_TOP_POS))
                .addTemporalMarker(()->{slider.setPosition(0.47);})
                .waitSeconds(1)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(1)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .waitSeconds(0.5)
                .build();
        TrajectorySequence drop5 = drive.trajectorySequenceBuilder(goToPickPos.end())
                .addTemporalMarker(()->extendTo(POSITIONS[HIGH_POLE]))
                .addTemporalMarker(()->setTurretDegree(-136))
                .lineToConstantHeading(new Vector2d(-36.2, 11.4))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->wrist.setPosition(WRIST_TOP_POS))
                .addTemporalMarker(()->{slider.setPosition(0.47);})
                .waitSeconds(1)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(1)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .waitSeconds(0.5)
                .build();



        TrajectorySequence goToPickPos2 = drive.trajectorySequenceBuilder(drop.end())
                .addTemporalMarker(()->wrist.setPosition(WRIST_INTAKE_POS))
                .addTemporalMarker(()->{setTurretDegree(1);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .addTemporalMarker(()->slider.setPosition(SLIDER_OUT_POS))
                .lineToConstantHeading(new Vector2d(-52.475, 13.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->extendTo((int)CONE_HEIGHTS[3]))
                .waitSeconds(0.5)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
                .waitSeconds(0.6)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(0.8)
                .addTemporalMarker(()->slider.setPosition(0.5))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence goToPickPos3 = drive.trajectorySequenceBuilder(drop.end())
                .addTemporalMarker(()->wrist.setPosition(WRIST_INTAKE_POS))
                .addTemporalMarker(()->{setTurretDegree(1);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->slider.setPosition(SLIDER_OUT_POS))
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .lineToConstantHeading(new Vector2d(-52.475, 13.4))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->extendTo((int)CONE_HEIGHTS[2]))
                .waitSeconds(0.5)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
                .waitSeconds(0.6)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(0.8)
                .addTemporalMarker(()->slider.setPosition(0.5))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence goToPickPos4 = drive.trajectorySequenceBuilder(drop.end())
                .addTemporalMarker(()->wrist.setPosition(WRIST_INTAKE_POS))
                .addTemporalMarker(()->{setTurretDegree(1);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->slider.setPosition(SLIDER_OUT_POS-0.1))
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .lineToConstantHeading(new Vector2d(-52.775, 13.3))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->extendTo((int)CONE_HEIGHTS[1]))
                .waitSeconds(0.5)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
                .waitSeconds(0.6)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(0.8)
                .addTemporalMarker(()->slider.setPosition(0.5))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence goToPickPos5 = drive.trajectorySequenceBuilder(drop.end())
                .addTemporalMarker(()->wrist.setPosition(WRIST_INTAKE_POS))
                .addTemporalMarker(()->{setTurretDegree(1);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->slider.setPosition(SLIDER_OUT_POS-0.1))
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_INTAKE_POSITION))
                .lineToConstantHeading(new Vector2d(-52.775, 13.2))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->extendTo((int)CONE_HEIGHTS[0]))
                .waitSeconds(0.5)
                .addTemporalMarker(()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
                .waitSeconds(0.6)
                .addTemporalMarker(()->slider.setPosition(0.5))
                .waitSeconds(0.2)
                .addTemporalMarker(()->extendTo(POSITIONS[MID_POLE]))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence goToPark2 = drive.trajectorySequenceBuilder(drop5.end())
                .addTemporalMarker(()->slider.setPosition(SLIDER_IN_POS))
                .waitSeconds(0.2)
                .addTemporalMarker(()->setTurretDegree(-90))
                .lineToConstantHeading(new Vector2d(-36, 12))
                .build();

        TrajectorySequence goToPark1 = drive.trajectorySequenceBuilder(drop5.end())
                .addTemporalMarker(()->slider.setPosition(SLIDER_IN_POS))
                .addTemporalMarker(()->wrist.setPosition(WRIST_TOP_POS))
                .waitSeconds(0.2)
                .addTemporalMarker(()->setTurretDegree(0))
                .lineToConstantHeading(new Vector2d(-12, 12))
                .build();

        TrajectorySequence goToPark3 = drive.trajectorySequenceBuilder(drop5.end())
                .addTemporalMarker(()->slider.setPosition(SLIDER_IN_POS))
                .waitSeconds(0.2)
                .addTemporalMarker(()->setTurretDegree(-90))
                .lineToConstantHeading(new Vector2d(-48, 12))
                .build();


//
//        TrajectorySequence goToPickPos3 = drive.trajectorySequenceBuilder(startToHighPole.end())
//                .addTemporalMarker(()->slider.setPosition(SLIDER_IN_POS))
//                .addTemporalMarker(()->{setTurretDegree(0);})
//                .addTemporalMarker(()->gripper.setPosition(GRIPPER_OPEN_POSITION))
//                .lineToConstantHeading(new Vector2d(-56, 12))
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->extendTo((int)CONE_HEIGHTS[2]))
//                .UNSTABLE_addTemporalMarkerOffset(1,()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
//                .waitSeconds(2)
//                .build();
//
//        TrajectorySequence goToPickPos4 = drive.trajectorySequenceBuilder(startToHighPole.end())
//                .addTemporalMarker(()->slider.setPosition(SLIDER_IN_POS))
//                .addTemporalMarker(()->{setTurretDegree(0);})
//                .addTemporalMarker(()->gripper.setPosition(GRIPPER_OPEN_POSITION))
//                .lineToConstantHeading(new Vector2d(-56, 12))
//                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->extendTo((int)CONE_HEIGHTS[1]))
//                .UNSTABLE_addTemporalMarkerOffset(1,()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
//                .waitSeconds(2)
//                .build();
//
//        TrajectorySequence goToPickPos5 = drive.trajectorySequenceBuilder(startToHighPole.end())
//                .addTemporalMarker(()->slider.setPosition(SLIDER_IN_POS))
//                .addTemporalMarker(()->{setTurretDegree(0);})
//
//                .addTemporalMarker(()->gripper.setPosition(GRIPPER_OPEN_POSITION))
//
//                .lineToConstantHeading(new Vector2d(-56, 12))
//                .UNSTABLE_addTemporalMarkerOffset(-1,()->extendTo((int)CONE_HEIGHTS[0]))
//
//                .UNSTABLE_addTemporalMarkerOffset(1,()->gripper.setPosition(GRIPPER_CLOSE_POSITION))
//
//                .waitSeconds(2)
//                .build();
//





        double position = 0;
        PIDFCoefficients leftMotorCoeff = elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients rightMotorCoeff = elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeInInit()){

            callWhileInInit();

            leftMotorCoeff.p = 10;
            leftMotorCoeff.d = 3;

            rightMotorCoeff.p = 10;
            rightMotorCoeff.d = 3;

            elevatorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, leftMotorCoeff);
            elevatorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, rightMotorCoeff);
            telemetry.addData("Left motor coefficients: ", elevatorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Right motor coefficients:", elevatorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.update();
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

            if(gamepad1.right_stick_button){
                wrist.setPosition(WRIST_INTAKE_POS);
            }
            else if(gamepad1.left_stick_button){
                wrist.setPosition(WRIST_DEFAULT_POS);
            }
            telemetry.addData("Position in mm:", elevatorLeft.getCurrentPosition() * (1/COUNTS_PER_MM));
            telemetry.addData("Position in counts:", elevatorLeft.getCurrentPosition());
//            telemetry.update();
            extendTo((int)position);
        }



        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendTo(0);
        wrist.setPosition(WRIST_DEFAULT_POS);
        slider.setPosition(SLIDER_IN_POS);
        position = 0;

        double degrees = 0;


        int parkingSpot = getParkingSpot();



        drive.followTrajectorySequence(startToHighPole);
        drive.followTrajectorySequence(goToPickPos);
        drive.followTrajectorySequence(drop);
        drive.followTrajectorySequence(goToPickPos2);
        drive.followTrajectorySequence(drop1);
//        drive.followTrajectorySequence(goToPickPos3);
//        drive.followTrajectorySequence(drop2);
//        drive.followTrajectorySequence(goToPickPos4);
//        drive.followTrajectorySequence(drop3);
//        drive.followTrajectorySequence(goToPickPos5);
//        drive.followTrajectorySequence(drop4);
        switch (parkingSpot){
            case 1:
                drive.followTrajectorySequence(goToPark1);
                break;
            case 3:
                drive.followTrajectorySequence(goToPark3);
                break;
            default:
                drive.followTrajectorySequence(goToPickPos3);
                drive.followTrajectorySequence(drop2);
                drive.followTrajectorySequence(goToPark2);
                break;
        }
        while (opModeIsActive()) {
            slider.setPosition(gamepad1.right_trigger);
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

            if(gamepad1.right_stick_button){
                wrist.setPosition(WRIST_INTAKE_POS);
            }
            else if(gamepad1.left_stick_button){
                wrist.setPosition(WRIST_DEFAULT_POS);
            }
            telemetry.addData("Parking Spot Number: ", parkingSpot);
            telemetry.addData("Position in mm:", elevatorLeft.getCurrentPosition() * (1/COUNTS_PER_MM));
            telemetry.addData("Position in counts:", elevatorLeft.getCurrentPosition());
//            telemetry.update();
            extendTo((int)position);
            telemetry.addData("Slider pos", slider.getPosition());
            telemetry.update();

        }
    }


    void setTurretDegree(double degree){
        turret.setTargetPosition((int)(COUNTS_PER_DEGREE*degree));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    void extendTo(int pos){
        elevatorLeft.setTargetPosition(pos);

        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(1);
        elevatorRight.setTargetPosition(pos);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
    }


    int getParkingSpot() {
        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest.id == ID_TAG_OF_INTEREST_1) {
            return 1;
        } else if (tagOfInterest.id == ID_TAG_OF_INTEREST_2) {
            return 2;
        } else if (tagOfInterest.id == ID_TAG_OF_INTEREST_3) {
            return 3;
        } else {
            return 2;
        }
    }

    void callWhileInInit() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound1 = false;
            boolean tagFound2 = false;
            boolean tagFound3 = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == ID_TAG_OF_INTEREST_1) {

                    tagOfInterest = tag;
                    tagFound1 = true;
                    break;
                } else if (tag.id == ID_TAG_OF_INTEREST_2) {
                    tagOfInterest = tag;
                    tagFound2 = true;
                    break;
                } else if (tag.id == ID_TAG_OF_INTEREST_3) {
                    tagOfInterest = tag;
                    tagFound3 = true;
                    break;
                }
            }

            if (tagFound1) {
                telemetry.addLine("Tag 1 is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else if (tagFound2) {
                telemetry.addLine("Tag 2 is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else if (tagFound3) {
                telemetry.addLine("Tag 3 is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
