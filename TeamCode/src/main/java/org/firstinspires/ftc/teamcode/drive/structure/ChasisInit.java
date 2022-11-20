package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChasisInit {

    public DcMotor BackLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor FrontLeft = null;
    public DcMotor BackRight = null;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        BackLeft = hwMap.get(DcMotor.class, "Back_Left");
        FrontRight = hwMap.get(DcMotor.class, "Front_Right");
        FrontLeft = hwMap.get(DcMotor.class, "Front_Left");
        BackRight = hwMap.get(DcMotor.class, "Back_Right");

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

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
}