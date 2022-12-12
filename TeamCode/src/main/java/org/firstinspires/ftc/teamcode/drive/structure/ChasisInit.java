package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChasisInit {

    public DcMotorEx BackLeft = null;
    public DcMotorEx FrontRight = null;
    public DcMotorEx FrontLeft = null;
    public DcMotorEx BackRight = null;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        BackLeft = hwMap.get(DcMotorEx.class, "Back_Left");
        FrontRight = hwMap.get(DcMotorEx.class, "Front_Right");
        FrontLeft = hwMap.get(DcMotorEx.class, "Front_Left");
        BackRight = hwMap.get(DcMotorEx.class, "Back_Right");

        BackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        FrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        BackRight.setDirection(DcMotorEx.Direction.REVERSE);


        BackLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

        public void  setMotortPower (double BackLeftPower ,double FrontRightPower,double FrontLeftPower,double BackRightPower)
    {
        BackLeft.setPower(BackLeftPower);
        FrontRight.setPower(FrontRightPower);
        FrontLeft.setPower(FrontLeftPower);
        BackRight.setPower(BackRightPower);

    }

}
