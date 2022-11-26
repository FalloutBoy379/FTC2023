package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class mechanisms {
    public static class Extension{

        private final double PULLEY_DIAMETER = 20.00; //mm
        private final double PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER * Math.PI;
        private final double EXTENSION_PER_REVOLUTION = PULLEY_CIRCUMFERENCE;
        private final double COUNTSPERREVOLUTION = 28.00;
        private final double EXTENSION_PER_COUNT = (1/COUNTSPERREVOLUTION)*EXTENSION_PER_REVOLUTION;
        private final double COUNT_PER_MM = 1/EXTENSION_PER_COUNT;

        public final int targetMillimeters = 0;

        TouchSensor homingSensor;

        DcMotorEx leftMotor = null;
        DcMotorEx rightMotor = null;

        Telemetry telemetry;

        public Extension(HardwareMap hardwareMap, Telemetry telemetry_local){
            this.telemetry = telemetry_local;
            this.leftMotor = hardwareMap.get(DcMotorEx.class, "leftExtension");
            this.rightMotor = hardwareMap.get(DcMotorEx.class, "rightExtension");
            this.homingSensor = hardwareMap.get(TouchSensor.class, "homingSensorExtension");

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void periodic(){
            telemetry.addData("Left motor position in millimeters: ", getMM()[0]);
            telemetry.addData("Right motor position in millimeters: ", getMM()[1]);
            telemetry.addData("Homing Sensor sttaus: ", this.homingSensor.isPressed());
            telemetry.addData("Motor current Left: ", getCurrents()[0]);
            telemetry.addData("Motor current Right: ", getCurrents()[1]);
//            extendTo(targetMillimeters);
        }


        public void extendTo(double pos){
            int position = (int)pos;
            this.leftMotor.setTargetPosition(position);
            this.rightMotor.setTargetPosition(position);
            this.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.leftMotor.setPower(1);
            this.rightMotor.setPower(1);
            this.leftMotor.setCurrentAlert(2, CurrentUnit.AMPS);
            this.rightMotor.setCurrentAlert(2, CurrentUnit.AMPS);
        }

        public void extendMM(double mm){
            double target = mm*COUNT_PER_MM;
            extendTo(target);
        }

        public void driveMotors(double power){
            this.leftMotor.setPower(power);
            this.rightMotor.setPower(power);
        }

        public void reset(){
            this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public double[] getCurrents(){
            return new double[]{this.leftMotor.getCurrent(CurrentUnit.MILLIAMPS), this.rightMotor.getCurrent(CurrentUnit.MILLIAMPS)};
        }

        public int[] getCounts(){
            return new int[]{this.leftMotor.getCurrentPosition(), this.rightMotor.getCurrentPosition()};
        }

        public double[] getMM(){
            return new double[]{this.leftMotor.getCurrentPosition()*EXTENSION_PER_COUNT, this.rightMotor.getCurrentPosition()*EXTENSION_PER_COUNT};
        }




    }
}
