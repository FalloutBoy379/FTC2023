package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class FTClibTeleop extends LinearOpMode {


    double COUNTS_PER_DEGREE = 5.423728813559322;
    double COUNTS_PER_MM = 4.7558;


    MotorEx turret;

    @Override
    public void runOpMode() throws InterruptedException {

        turret = new MotorEx(hardwareMap, "turret");

        turret.setPositionCoefficient(0.1);

        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setPositionTolerance(2);

//        turret.setDistancePerPulse(1.0/COUNTS_PER_DEGREE);
//        turret.setTargetDistance(0);
        telemetry.addData("Kp of turret: ", turret.getPositionCoefficient());
        telemetry.update();

        waitForStart();



        while(opModeIsActive()){
            if(gamepad1.a){
                turret.setTargetPosition((int)(COUNTS_PER_DEGREE * 90));
            }
            else if(gamepad1.b){
                turret.setTargetPosition(0);
            }

            if(gamepad1.right_bumper){
                turret.setPositionCoefficient(turret.getPositionCoefficient() + 0.1);
            }
            else if(gamepad1.left_bumper){
                turret.setPositionCoefficient(turret.getPositionCoefficient() - 0.1);
            }
            telemetry.addData("Position: ", turret.getCurrentPosition());
            telemetry.addData("Kp of turret: ", turret.getPositionCoefficient());
            telemetry.update();
            turret.set(1);
        }
    }

    void setTurretDegree(double degree){
        turret.setTargetPosition((int)(COUNTS_PER_DEGREE*degree));
    }
}
