package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;


/** Testing some funky shit with motion profile */

public class NoPermaMP {

    public double firstReturn = 0;
    public double secondReturn = 0;
    public double thirdReturn = 0;
    public double max_vel;
    public double acc_dt;
    public double calc;

    public double getRekt = 1;

    public double motion_profile(double max_acceleration, double max_velocity, double distance,double current_dt) {

        double acceleration_dt = max_velocity / max_acceleration;

        double halfway_distance = distance / 2;

        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            calc = halfway_distance / (0.5 * max_acceleration);
            if(calc < 0){
                calc *= -1;
                getRekt = -1;
            }else{
                getRekt = 1;
            }

            acceleration_dt = Math.sqrt(calc);
        }



        acc_dt = acceleration_dt;

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // acceleration_dt = Math.sqrt(distance/max_acceleration);

        max_velocity = max_acceleration * acceleration_dt;

        if(max_velocity == 0)max_velocity = 1;

        max_vel = max_velocity;

        double deacceleration_dt;
        deacceleration_dt = acceleration_dt;

        double cruise_distance;
        cruise_distance = distance - acceleration_distance;
        double cruise_dt;
        cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time;
        deacceleration_time = acceleration_dt + cruise_dt;

        double entire_dt;
        entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        if (current_dt > entire_dt) {
            firstReturn = distance;
            return distance;
        }

        double cruise_current_dt;
        if (current_dt < acceleration_dt) {
            return (0.5 * max_acceleration * Math.pow(current_dt, 2)) * getRekt;
        } else if (current_dt < deacceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_current_dt = current_dt - acceleration_dt;
            return (acceleration_distance + max_velocity * cruise_current_dt) * getRekt;
        } else {

            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deacceleration_time = current_dt - deacceleration_time;

            return (acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * Math.pow(deacceleration_time,2)) * getRekt;
        }
    }
}
