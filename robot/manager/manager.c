/****************************************
 *                                     
 * The purpose of this program is to do 
 * the following:                       
 *                                      
 * 1) Open and store all the arduinos   
 *    in linked list or hashtable, then 
 *    map each arduino to an id.
 * Sensors:
 * 2) Extract data from each id given
 *    the following format:
 *      [id data_1 data_2 ... data_n]
 * 3) Send those data to a user when they
 *    ask for it through get_<sensor>().
 * Motor:
 * 4) Provide a way for users to issue
 *    direct control through
 *    set_<motor>()
 * 5) Provide a way for users to issue
 *    abstracted control through:
 *    forward()
 *    backward()
 *    turn_left()
 *    turn_right()
 *    stop()
 *    grab()
 *    release()
 *    open_claw()
 *    close_claw()
 * 6) Send those commands to the arduino
 *    through the following format:
 *      [data_1 data_2 ... data_n]\n
 * Miscellaneous:
 * 7) Provide a list of properties:
 *    can_turn_yaw    (on the y-axis)
 *                    (left/right)
 *    can_turn_pitch  (on the x-axis)
 *    can_turn_roll   (on the z-axis)
 *    can_move_y      (forward/backward)
 *    can_move_x      (strafe left/right)
 *    can_move_z      (up/down)
 *    num_arms
 *    num_joints_arm
 *    has_claw
 *    has_conveyor
 *    has_custom
 *    function names
 *
 ***************************************/
