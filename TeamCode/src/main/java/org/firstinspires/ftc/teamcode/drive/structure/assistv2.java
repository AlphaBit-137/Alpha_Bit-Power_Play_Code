package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class assistv2 {

    Slider slider = new Slider();

    public final int Ground_Junction = 150;
    public final int Medium_Pole = 4232;
    public final int High_Pole = 7650;

    public void init(HardwareMap ahwMap){
        slider.init(ahwMap);
    }

    LVLS lvls = LVLS.RESET;

    enum LVLS{
        LVL1,
        LVL2,
        BACK,
        RESET
    }

    public void update(){
        switch(lvls){
            case LVL1:
                GoToLvl(Medium_Pole);
                break;
            case LVL2:
                GoToLvl(High_Pole);
                break;
            case BACK:
                GoToLvl(Ground_Junction);
                break;
            case RESET:
                slider.Slider_Reset();
                break;
        }
    }

    public void GoToLvl( int pos){
        slider.Slider_GoToPos(pos);
        if(!slider.SliderBUSY()) SwitchToReset();
    }

    public void SwitchToLvl1() {
        lvls = LVLS.LVL1;
    }
    public void SwitchToLvl2() {
        lvls = LVLS.LVL2;
    }
    public void SwitchToReset() {
        lvls = LVLS.RESET;
    }
    public void SwitchToBACK(){lvls = LVLS.BACK;}
}
