package org.firstinspires.ftc.teamcode.drive.structure;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Color_Sensor {

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    float saturation;

    boolean active = false;


    final double SCALE_FACTOR = 255;

    int relativeLayoutId;

    int green, red, blue;

    int cas = 3;

    View relativeLayout;

    public void init(HardwareMap hardmap) {

        sensorColor = hardmap.get(ColorSensor.class, "sensor_color_distance");

        sensorDistance = hardmap.get(DistanceSensor.class, "sensor_color_distance");

        relativeLayoutId = hardmap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardmap.appContext.getPackageName());
        relativeLayout = ((Activity) hardmap.appContext).findViewById(relativeLayoutId);
    }

    float hsvValues[] = {0F, 0F, 0F};

    float values[] = hsvValues;


    public void update() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        saturation = hsvValues[1];

        green=sensorColor.green();
        red = sensorColor.red();
        blue = sensorColor.blue();


    }

    public int whatColorIsIt() {
        if (green > (blue + red) / 1.40) {
            return 3; //Neither blue nor red
        } else if (red < blue && saturation >= 0.25) {
            return 2; //blue
        } else {
            return 1; //red
        }
    }
    public int whatColorIsIt2()
    {
        if(green < red || green < blue)
        {
            if(red>blue)return 1;
            else if(blue>red)return 2;
        }else return 3;
        return 0;
    }

}