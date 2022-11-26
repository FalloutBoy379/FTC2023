package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Input {

    public static class Gripper{
        private double openPosition = 0.45;
        private double closedPosition = 0.9;
        Servo motor = null;
        public Gripper(HardwareMap hardwareMap){
            motor = hardwareMap.get(Servo.class, "intakeServo");
        }

        public void openGripper(){
           motor.setPosition(openPosition);
        }

        public void closeGripper(){
            motor.setPosition(closedPosition);
        }

        public double getPos(){
            return motor.getPosition();
        }

    }


    public static class Wrist{
        private double inPosition = 0;
        private double outPosition = 1;
        Servo motor = null;
        public Wrist(HardwareMap hardwareMap){
            motor = hardwareMap.get(Servo.class, "intakeWrist");
        }

        public void goOutside(){
            motor.setPosition(outPosition);
        }

        public void goInside(){
            motor.setPosition(inPosition);
        }
    }


    public static class Lift{
        DcMotorEx motor = null;
        int maxExtension = 850;
        int minExtension = 0;

        public int[] conePositions = {0,0,0,0,0};
        public Lift(HardwareMap hardwareMap){
            motor = hardwareMap.get(DcMotorEx.class, "liftMotor");
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public void reset(){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public double getPosition(){
            return motor.getCurrentPosition();
        }

        public void extendTo(int pos){
            if(pos>maxExtension){
                pos =  maxExtension;
            }
            else if(pos < minExtension){
                pos = minExtension;
            }
            motor.setTargetPosition(pos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }

        public void driveMotor(double power){
            if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            motor.setPower(power);
        }
    }
}
