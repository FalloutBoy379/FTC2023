package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp
@Config
public class test0 extends LinearOpMode {
    public static double lift_setpoint = 0;
    DcMotorEx leftExtensionMotor, rightExtensionMotor, leftSlideMotor, rightSlideMotor;
    Servo leftTransferServo, rightTransferServo, gripperYawServo, gripperGripServo;

    double liftSetpoint = 0;
    double slideSetpoint = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        leftExtensionMotor = hardwareMap.get(DcMotorEx.class, "leftExtensionMotor");
        rightExtensionMotor = hardwareMap.get(DcMotorEx.class, "rightExtensionMotor");
//        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
//        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
//        leftTransferServo = hardwareMap.get(Servo.class, "leftTransferServo");
//        rightTransferServo = hardwareMap.get(Servo.class, "rightTransferServo");
//        gripperYawServo = hardwareMap.get(Servo.class, "gripperYawServo");
//        gripperGripServo = hardwareMap.get(Servo.class, "gripperGripServo");

        PIDFCoefficients pidf_lift_coeff = leftExtensionMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidf_slide_coeff = rightExtensionMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        PIDFController pidf_lift = new PIDFController(pidf_lift_coeff.p, pidf_lift_coeff.i, pidf_lift_coeff.d, pidf_lift_coeff.f);
        PIDFController pidf_slide = new PIDFController(pidf_slide_coeff.p, pidf_slide_coeff.i, pidf_slide_coeff.d, pidf_slide_coeff.f);

        leftExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            extendTo((int)lift_setpoint);
//            double leftMotorExtensionCurrent = leftExtensionMotor.getCurrent(CurrentUnit.MILLIAMPS);
//            double rightMotorExtensionCurrent = rightExtensionMotor.getCurrent(CurrentUnit.MILLIAMPS);
//            double errorCurrent = leftMotorExtensionCurrent - rightMotorExtensionCurrent;
//            double output = pidf_lift.calculate(leftExtensionMotor.getCurrentPosition(), lift_setpoint);
//            output = Range.clip(output, -1, 1);
//            leftExtensionMotor.setPower(output);
//            rightExtensionMotor.setPower(-output);
        }
    }

    public void extendTo(int counts){
        leftExtensionMotor.setTargetPosition(counts);
        rightExtensionMotor.setTargetPosition(counts);
        leftExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtensionMotor.setPower(1);
        rightExtensionMotor.setPower(1);
    }
}
