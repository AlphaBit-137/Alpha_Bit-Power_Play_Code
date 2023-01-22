package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

public class MotionProfile {

    public class Motion_Profile_Skeleton {

        public double motion_profile(double max_acceleration, double max_velocity, double distance,double vel_t) {


            double  acceleration_dt = max_velocity / max_acceleration;
            double halfway_distance = distance / 2;
            double acceleration_distance = 0.5 * max_acceleration * acceleration_dt * 2;

            if (acceleration_distance > halfway_distance);
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

            max_velocity = max_acceleration * acceleration_dt;


            double  deacceleration_dt = acceleration_dt;


            double cruise_distance = distance - acceleration_distance;

            double cruise_dt = cruise_distance / max_velocity;

            double deacceleration_time = acceleration_dt + cruise_dt;


            double  entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
            if (vel_t > entire_dt)
                return distance;


            if (vel_t < acceleration_dt){
                return 0.5 * max_acceleration * Math.pow(vel_t,2);
            }else if (vel_t < deacceleration_time) {
                acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
                double cruise_current_dt = vel_t - acceleration_dt;
                return acceleration_distance + max_velocity * cruise_current_dt;
            } else {
                acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
                cruise_distance = max_velocity * cruise_dt;
                deacceleration_time = vel_t - deacceleration_time;


                return Math.pow(acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * deacceleration_time,2);
            }
        }

    }

}
