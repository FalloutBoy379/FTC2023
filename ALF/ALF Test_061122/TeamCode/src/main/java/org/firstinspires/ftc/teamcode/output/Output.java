package org.firstinspires.ftc.teamcode.output;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Output {


    //------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------

    public static class Extension{
        DcMotorEx motorL = null;
        DcMotorEx motorR = null;

        public int minExtension = 0;
        public int maxExtension = 5000;
        private double maxPower = 1;

        public Extension(HardwareMap hardwareMap){
            motorL = hardwareMap.get(DcMotorEx.class, "extensionMotorL");
            motorR = hardwareMap.get(DcMotorEx.class, "extensionMotorR");
            motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public double[] getCurrent(){
            double[] returnVal = {motorL.getCurrent(CurrentUnit.MILLIAMPS), motorR.getCurrent(CurrentUnit.MILLIAMPS)};
            return  returnVal;
        }

        public double[] getPosition(){
            double[] returnVal = {motorL.getCurrentPosition(), motorR.getCurrentPosition()};
            return returnVal;
        }

        public double[] getTargetPos(){
            double[] returnVal = {motorL.getTargetPosition(), motorR.getTargetPosition()};
            return returnVal;
        }

        public void extendTo(int pos){
            if(Math.abs(pos)>Math.abs(maxExtension)){
                pos =  maxExtension;

            }
            else if(Math.abs(pos) < Math.abs(minExtension)){
                pos = minExtension;
            }
            motorL.setTargetPosition(pos);
            motorR.setTargetPosition(pos);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(maxPower);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(maxPower);
        }

        public void driveMotor(double power){
            if(motorL.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(motorR.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(power > maxPower){
                power = maxPower;
            }
            else if(power < -maxPower){
                power = -maxPower;
            }
            motorR.setPower(power);
            motorL.setPower(power);
        }

        public void reset(){
            motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    //------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------

    public static class Gripper {
        private double openPosition = 0;
        private double closedPosition = 0.8;
        String open = "Open";
        String closed = "Closed";
        Servo motor = null;

        public Gripper(HardwareMap hardwareMap) {
            motor = hardwareMap.get(Servo.class, "outputGripper");
            openGripper();
        }
        //0.9, 0.45 Intake

        String gripperState = closed;


        public void openGripper() {
            gripperState = open;
            motor.setPosition(openPosition);
        }

        public void closeGripper() {
            gripperState = closed;
            motor.setPosition(closedPosition);
        }

        public void toggleGripper() {
            if (gripperState == closed) {
                openGripper();
            } else if (gripperState == open) {
                closeGripper();
            }
        }

    }


    //------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------

    public static class Wrist {
        Servo motorL = null;
        Servo motorR = null;

        double outsidePos = 1;
        double insidePos = 0;
        double midPos = 0.5;

        public Wrist(HardwareMap hardwareMap) {
            motorL = hardwareMap.get(Servo.class, "motorL");
            motorR = hardwareMap.get(Servo.class, "motorR");
        }

        public void goTo(double pos){
            motorL.setPosition(pos);
            motorR.setPosition(1 - pos);

        }

        public void goInside(){
            goTo(insidePos);
        }

        public void goOutside(){
            goTo(outsidePos);
        }

        public void midPos(){
            goTo(midPos);
        }
    }
}
