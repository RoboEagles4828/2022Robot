package frc.robot.Constants;

public class Speeds {
    public static double manual_shooter_speed_high = 0.40;///When manually controlled 0.69 for high, xx for low
    public static double manual_shooter_speed_low = 0.40;//Low shooter speed, not tested
    public static double wall_shooter_speed = -7.4;//Flush against wall shooting
    public static double manual_shooter_back_speed = 1.0;
    

    public static double index_conveyor_speed = 0.5;
    public static double manual_conveyor_speed = 0.8;
    public static double conveyor_shoot_speed = 0.8;

    public static double manual_intake_speed = 0.7;

    public static double manual_climber_speed_up = 0.8;
    public static double manual_climber_speed_down = 0.8;
    public static double manual_left_climber_speed_up = 0.7;
    public static double manual_left_climber_speed_down = 0.9;
    public static double manual_right_climber_speed_up = 0.6;
    public static double manual_right_climber_speed_down = 0.8;

    public static double auto_drive_speed = 0.6;
    public static double auto_intake_speed = 0.7;
    public static double auto_field_shoot_speed = 1.0;
    public static double auto_turn_speed = 0.3;

    public static double shooter_volt_far = -6.5;//-7.4, -4.6 for 26inch
    public static double shooter_volt_close = -4.6;
    public static double shooter_back_volt_far = -12;//Ratio is 5:12 for front:back for little back spin,-11.45 for 26 inch
    public static double shooter_back_volt_close = -11.45;
    public static double raw_shooter_vel_far = 16000;//14750
    public static double raw_shooter_vel_close = 10500;//10500
    public static double raw_shooter_back_vel = 30000;//34000, 28000

    public static double spacing_speed = 0.3;
}
