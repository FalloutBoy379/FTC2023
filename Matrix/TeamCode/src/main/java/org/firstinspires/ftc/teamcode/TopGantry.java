package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TopGantry {

    public static class Slider{
        public double MM_MIN = 0;
        public double MM_MAX = 11;
        public double MM_RANGE = Math.abs(MM_MAX - MM_MIN);

        ServoEx sliderServo = null;

        public Slider(HardwareMap hardwareMap){
            sliderServo = hardwareMap.get(ServoEx.class, "slider");
        }

        public double map(double x, double in_min, double in_max, double out_min, double out_max){
            return ((x-in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min);
        }

        public void sliderMM(double pos){
            pos = map(pos, 0, 11, 0, 1);
            sliderServo.setPosition(pos);
        }
    }
}
