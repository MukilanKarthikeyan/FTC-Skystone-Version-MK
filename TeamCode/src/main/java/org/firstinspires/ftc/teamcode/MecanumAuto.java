package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
@Autonomous
public class MecanumAuto extends LinearOpMode {

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    BNO055IMU imu;
    Orientation angles;


    public void runOpMode() throws InterruptedException {
            leftFront = hardwareMap.get(DcMotor.class, "lf");
            rightFront = hardwareMap.get(DcMotor.class, "rf");
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack = hardwareMap.get(DcMotor.class, "lb");
            rightBack = hardwareMap.get(DcMotor.class, "rb");
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void fwd(int dist){

    }
    public void rever(int dist){

    }
    public void strafe(int dist){
        // if dist is positive strafes right, if dist is negative strafes left
    }

    public void drive(int x, int y, double turn) {
        //calculations for the power to be applied to each wheel
        // NOTE: these are what would be used for the telOP inputs form controller
        double leftFrontVal = y + turn + x;
        double leftBackVal = y + turn - x;
        double rightFrontVal = y - turn - x;
        double rightBackVal = y - turn + x;

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }
}


