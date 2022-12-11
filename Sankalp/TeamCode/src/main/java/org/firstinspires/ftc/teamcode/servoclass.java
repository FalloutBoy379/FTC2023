package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoclass {
    public Servo servo1;
    public void init(HardwareMap hardwareMap) {

        servo1=hardwareMap.servo.get("servo1");
    }

    public void setservo(Servo servo,double pos){
        servo.setPosition(pos);
    }

}
