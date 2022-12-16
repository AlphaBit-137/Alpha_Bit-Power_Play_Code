package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *
 * Using this since i got no time to live
 *
 */


public class SliderV2 {

    public DcMotor slider;

    Positions SliderPosition = Positions.STOP;
    HardwareMap hwMap = null;

    public enum Positions{
        UP,
        DOWN,
        STOP
    }

    public void init(HardwareMap ahwMap) {
        // Define and Initialize Motors

        hwMap = ahwMap;
        slider = hwMap.get(DcMotor.class,"Slider");
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setDirection(DcMotor.Direction.FORWARD);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setPower(0);
    }


    public void Slider_GoToPos(int position)
    {
        slider.setTargetPosition(position);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Reset_Encoder(){
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int Slider_GetPosition(){
        return slider.getCurrentPosition();
    }

    public void update(){
        switch (SliderPosition){
            case UP:{
                slider.setPower(-0.5);
                break;
            }
            case DOWN:{
                slider.setPower(0.5);
                break;
            }
            case STOP:{
                slider.setPower(0);
                break;
            }
        }
    }

    public void switchToSliderUp() {SliderPosition = Positions.UP;}

    public void switchToSliderDown() {SliderPosition = Positions.DOWN;}

    public void switchToSliderSTOP() {SliderPosition = Positions.STOP;}

    public boolean SliderBUSY() {return slider.isBusy();}

}
