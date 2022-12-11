//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//public class AutoMechanism extends LinearOpMode {
//
//    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(180));
//
//    Trajectory startToDropPreload = new TrajectoryBuilder(startPose)
//            .lineToConstantHeading(new Vector2d())
//    .build()
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        periodic();
//    }
//
//
//
//
//    void periodic(){
//
//        telemetry.update();
//    }
//}
