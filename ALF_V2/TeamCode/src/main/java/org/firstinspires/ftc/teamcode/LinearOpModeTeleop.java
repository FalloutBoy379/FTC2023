package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LinearOpModeTeleop extends LinearOpMode {

    mechanisms.Extension extension;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftExtension = hardwareMap.get(DcMotorEx.class, "leftExtension");
        DcMotorEx rightExtension = hardwareMap.get(DcMotorEx.class, "rightExtension");


        waitForStart();

        while(opModeIsActive()){
            leftExtension.setPower(gamepad1.left_stick_y);
            rightExtension.setPower(gamepad1.left_stick_y);
        }
    }
}
