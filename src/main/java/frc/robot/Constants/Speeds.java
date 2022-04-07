package frc.robot.Constants;

public class Speeds {

    public static double index_conveyor_speed = 0.5;
    public static double manual_conveyor_speed = 0.8;
    public static double conveyor_shoot_speed = 0.8;

    public static double manual_intake_speed = 0.7;

    public static double manual_climber_speed_up = 0.8;
    public static double manual_climber_speed_down = 0.8;
    public static double manual_left_climber_speed_up = 1.0; //0.7;
    public static double manual_left_climber_speed_down = 1.0; //0.9;
    public static double manual_right_climber_speed_up = 1.0; //0.6;
    public static double manual_right_climber_speed_down = 1.0; //0.8;

    public static double auto_drive_speed = 0.6;
    public static double auto_intake_speed = 0.7;
    public static double auto_field_shoot_speed = 1.0;
    public static double auto_turn_speed = 0.4;

    public static double vel_threshold = 0.95;
    public static double shooter_volt_far = -3.7;
    public static double shooter_volt_close = -3.9;
    public static double idle_shooter_speed = -3;
    public static double shooter_back_volt_far = -11;//Ratio is 5:12 for front:back for little back spin,-11.45 for 26 inch
    public static double shooter_back_volt_close = -7;
    public static double raw_shooter_vel_far = vel_threshold*(2837.05*shooter_back_volt_far-1140.09);
    public static double raw_shooter_vel_close = vel_threshold*(2837.05*shooter_back_volt_close-1140.09);
    public static double raw_shooter_back_vel = vel_threshold*(3058.85*shooter_back_volt_far-2225.71);
}
