package frc.robot;

public class Constants{
    //Ports
    public static int front_left_port = 4;
    public static int back_left_port = 3;
    public static int front_right_port = 2;
    public static int back_right_port = 1;

    public static int proxy_lower = 9;
    public static int proxy_upper = 8;
    public static int hall_left = 7;
    public static int hall_right = 6;

    public static int shooter_port = 14;
    
    public static int conveyor_port = 15;

    public static int intake_port = 12;

    public static int left_climber_port = 20;
    public static int right_climber_port = 17;

    public static int intake_servo_port = 9;
    public static int shooter_servo_0_port = 0;
    public static int shooter_servo_1_port = 1;
    
    public static int joystick_0_port = 0;
    public static int joystick_1_port = 1;

    //Speeds
    public static double manual_shooter_speed_high = 0.65;///When manually controlled 0.69 for high, xx for low
    public static double manual_shooter_speed_low = 0.40;//Low shooter speed, not tested
    public static double wall_shooter_speed = 0.65;//Flush against wall shooting

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
    public static double auto_turn_speed = 0.1;

    //Buttons
    public static int manual_shoot_button = 1;
    
    public static int manual_intake_button = 5;
    public static int manual_intake_button_1 = 11;
    public static int manual_rev_intake_button = 6;

    public static int manual_conveyor_button = 3;
    public static int manual_rev_conveyor_button = 4;

    public static int manual_climber_button = 7;
    public static int manual_rev_climber_button = 8;
    public static int manual_left_climber_button = 9;
    public static int manual_rev_left_climber_button = 11;
    public static int manual_right_climber_button = 10;
    public static int manual_rev_right_climber_button = 12;

    public static int intake_servo_button = 12;
    public static int shooter_servo_button = 10;

    public static int space_shoot_button = 9;
    
    public static double encoder_ratio = 1375;//Number of raw values per inch

    //Auto Distances/Values
    public static double wall_to_ball = 160;//Distance from wall to ball in straight line

    public static double default_drive = 80;
    public static double bottom_tarmac_to_ball = 43;
    public static double bottom_ball_to_pivot = 75;
    public static double bottom_pivot_to_wall = 48;
    public static double bottom_pivot_angle = 65;

    public static double top_tarmac_to_ball = 36;
    public static double top_ball_to_wall = 110;
    

    public static double wall_shoot_time = 4;
    public static double conveyor_delay = 1.5;
    public static double conveyor_first_delay = 2.5;
    public static double intake_ball_time = 0.5;
    public static double ball_to_wall = 120;//Distance from ball to wall, higher because pushes against wall to become flush
    public static double stop_dt_time = 0.25;
    public static double index_time = 0.23;

    public static double shooter_vel = 0;
    public static double shooter_volt = -7.4;

    public static double spacing = 6;
    public static double spacing_speed = 0.3;
}