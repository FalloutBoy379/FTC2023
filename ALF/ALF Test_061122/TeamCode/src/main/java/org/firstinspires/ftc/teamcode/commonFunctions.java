package org.firstinspires.ftc.teamcode;

public class commonFunctions {
    public static double toggleVariable(double var){
        if(var == 0){
            return 1;
        }
        else if(var == 1){
            return 0;
        }

        return 0;
    }

}
