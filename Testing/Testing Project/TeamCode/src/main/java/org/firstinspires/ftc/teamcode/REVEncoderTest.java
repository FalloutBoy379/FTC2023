package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "EncoderTest")
public class REVEncoderTest extends LinearOpMode {

    DcMotorEx encoder = null;
    public void runOpMode(){
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        encoder = hardwareMap.get(DcMotorEx.class, "encoder0");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        telemetry.addData("Status: ", "Running");
        telemetry.update();
        while(opModeIsActive()){
            telemetry.addData("Encoder count: ", encoder.getCurrentPosition());
            telemetry.update();
        }

    }
}
