package org.firstinspires.ftc.teamcode.Mechanisms.Hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorTIS {

    public double MAX_CURRENT = 0;

    public enum RUNMODE{
        POSITION,
        VELOCITY
    }

    private HardwareMap localHardwareMap;
    private Telemetry localTelemetry;
    private String name;
    private DcMotorEx motor;

    public double Kp = 0;
    public double Kd = 0;
    public double Kf = 0;
    public double Ki = 0;

    private PIDFController controller = new PIDFController(Kp, Kd, Ki, Kf);

    public DcMotorTIS(HardwareMap hardwareMap, Telemetry telemetry, String deviceName){
        name = deviceName;
        localTelemetry = telemetry;
        localHardwareMap = hardwareMap;
        motor = localHardwareMap.get(DcMotorEx.class, name);
    }

    public void setMaximumCurrentWarning(double current){
        MAX_CURRENT = current;
    }

    public void setCoeffecients(double kp, double kd, double ki, double kf){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        Kf = kf;
        controller.setPIDF(Kp, Ki, Kd, Kf);
    }

    public void configZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public void setTargetPosition(double target){
        controller.setSetPoint(target);
    }

    public void printInfo(){
        String caption = captionConstructor(name, "Current");
        localTelemetry.addData(caption, getCurrent(CurrentUnit.MILLIAMPS));
        caption = captionConstructor(name, "Position");
        localTelemetry.addData(caption, getPosition());
        if(getCurrent(CurrentUnit.AMPS) > MAX_CURRENT){
            localTelemetry.addLine("<p style=\"color:orange\">" + name + " is over Current Limit</p>");
        }
    }

    private String captionConstructor(String nameOfDevice, String quantity){
        return (nameOfDevice+""+quantity+": ");
    }

    public void set(RUNMODE mode, double power){
        if(mode == RUNMODE.POSITION){
            double output = controller.calculate(getPosition());
            output = Range.clip(output, -power, power);
            motor.setPower(output);
        }
        else{
            motor.setPower(power);
        }
    }

    public void disable(){
        motor.setMotorDisable();
    }

    public int getPosition(){
        return motor.getCurrentPosition();
    }

    public double getCurrent(CurrentUnit unit){
        return motor.getCurrent(unit);
    }


}
