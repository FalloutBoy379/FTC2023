package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mechanisms {
    public static class Elevator{
        private DcMotorEx elevatorLeft = null;
        private DcMotorEx elevatorRight = null;

        public final int GRIPPING_POSITION = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;

        public final int[] POSITIONS = {520, -9040, -1970, -2800};

        public final double[] CONE_HEIGHTS = {POSITIONS[GRIPPING_POSITION], POSITIONS[GRIPPING_POSITION]-132.674, POSITIONS[GRIPPING_POSITION]-(2*132.674), POSITIONS[GRIPPING_POSITION]-(3*132.674), POSITIONS[GRIPPING_POSITION]-(4*132.674)};

        public final int MAX_EXTENSION = POSITIONS[HIGH_POLE];
        public final int MIN_EXTENSION = POSITIONS[GRIPPING_POSITION];

        public Elevator(HardwareMap hardwareMap){
            elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
            elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

            elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        public void extendTo(int pos){
            if(Math.abs(pos) > Math.abs(MAX_EXTENSION)){
                pos = MAX_EXTENSION;
            }
            else if(Math.abs(pos) < Math.abs(MIN_EXTENSION)){
                pos = MIN_EXTENSION;
            }
        }
    }
}
