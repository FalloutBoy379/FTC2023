//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//
//public class Turret extends SubsystemBase {
//    private final double COUNTS_PER_DEGREE = 5.423728813559322;
//    MotorEx turret;
//    DcMotorEx turretMotor = turret.motorEx;
//    Telemetry telemetry;
//
//    @Override
//
//    public void periodic() {
//        telemetry.addLine("---------------------TURRET----------------------");
//        telemetry.addData("Position: ", turret.getCurrentPosition());
//        telemetry.addData("Current: ", turretMotor.getCurrent(CurrentUnit.MILLIAMPS));
//        telemetry.update();
//    }
//
//    public Turret(final HardwareMap hardwareMap, Telemetry telemetry){
//        this.telemetry = telemetry;
//        turret = new MotorEx(hardwareMap, "turret");
//        turret.setPositionCoefficient(0.1);
//        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        turret.setRunMode(Motor.RunMode.PositionControl);
//        turret.setPositionTolerance(2);
//        setTurretDegree(0);
//    }
//
//
//    public void setTurretDegree(double degree){
//        turret.setTargetPosition((int)(COUNTS_PER_DEGREE*degree));
//    }
//
//    Trajectory traj = mysteryRobot.drive.trajectoryBuilder(new Pose2d(-36.8,62.4, Math.toRadians(-90)), Math.toRadians(-90))
//            .splineToSplineHeading(new Pose2d(-36.4,12, Math.toRadians(-49)), Math.toRadians(-49))
//            .splineToSplineHeading(new Pose2d(-66,12, Math.toRadians(-30)), Math.toRadians(-30))
//            .splineToSplineHeading(new Pose2d(-12.4,13.2, Math.toRadians(41)), Math.toRadians(41))
//}
