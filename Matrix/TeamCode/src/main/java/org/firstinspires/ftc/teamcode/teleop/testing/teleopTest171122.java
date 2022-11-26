package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;


@TeleOp
public class teleopTest171122 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(36, -60, 0));



        waitForStart();

        while(opModeIsActive()){
            drive.update();


            Pose2d robotPose = drive.getPoseEstimate();

            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-robotPose.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));

            telemetry.addData("X", robotPose.getX());
            telemetry.addData("Y", robotPose.getY());
            telemetry.addData("Heading", robotPose.getHeading());
            telemetry.update();
        }
    }
}
