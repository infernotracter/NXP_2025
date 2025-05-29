# ----------------- PID算法 -----------------
#位置式
def pid_controller(now, target, kp, ki, kd, sum_err, last_err):
    error = target - now
    sum_err += error
    derivative = error - last_err
    output = kp * error + ki * sum_err + kd * derivative
    return output, sum_err, error

#增量式
def pid_increment(now, target, kp, ki, kd, last_err, prev_err):
    error = target - now
    increment = kp*(error - last_err) + ki*error + kd*(error - 2*last_err + prev_err)
    last_err = error
    prev_err = last_err
    return increment, last_err, prev_err

# ----------------- 全局变量 -----------------
gyro_bias_y = 0
gyro_bias_z = 0
pitch_vel = 0
vel_disturbance = 0
last_turn_out = 0
real_speed = 0
acc_x = 0
gyro_y = 0
pwm_l = 0
pwm_r = 0
acc_z = 0
last_error2 = 0


target_speed = 0
balance_angle = 0
vel_kp = 0
vel_ki = 0
vel_kd = 0
angle_kp = 0
angle_ki = 0
angle_kd = 0
speed_kp = 0
speed_ki = 0
speed_kd = 0

now_speed = 0
counter_speed = 0
counter_angle = 0
speed_sum_error = 0
speed_last_error = 0
angle_sum_error = 0
angle_last_error = 0
vel_last_error = 0
vel_prev_error = 0
turn_sum_error = 0
turn_last_error = 0
turn_output = 0
pwm = 0
current_angle = 0
# ----------------- 控制回调函数 -----------------
def vel_loop_callback(pit0):
    global pwm, current_angle, turn_output
    global vel_last_error, vel_prev_error
    global turn_sum_error, turn_last_error
    global vel_kp, vel_ki, vel_kd
    global angle_disturbance, target_speed
    global speed_kp, speed_ki, speed_kd
    global speed_sum_error, speed_last_error
    global balance_angle, angle_kp, angle_ki, angle_kd
    global angle_sum_error, angle_last_error
    global yaw_vel, counter_speed, counter_angle
    global imu, gyro_bias_y, gyro_bias_z
    global motor_l, motor_r, last_value
    global protect_flag, imu_data, now_speed

    imu_data = imu.get()
    acc_x = imu_data[1]
    acc_z = imu_data[2]
    gyro_y = imu_data[3]
    gyro_z = imu_data[5]

    current_angle = update_filter(acc_x, gyro_y)
    yaw_vel = (gyro_z - gyro_bias_z) * 0.07
    pitch_vel = (gyro_y - gyro_bias_y) * 0.07

    counter_speed += 1
    if counter_speed >= 50:
        counter_speed = 0
        left = -encoder_left.get()
        right = -encoder_right.get()

        max_speed = 2500
        left = max(min(left, max_speed), -max_speed)
        right = max(min(right, max_speed), -max_speed)
        
        now_speed = now_speed*0.1 + ((left + right) / 2)*0.9

        angle_disturbance, speed_sum_error, speed_last_error = pid_controller(
            now_speed, target_speed, 
            speed_kp, speed_ki, speed_kd,
            speed_sum_error, speed_last_error
        )
        angle_disturbance = max(min(angle_disturbance, 450), -450)

    target_angle = balance_angle - angle_disturbance

    # 外环控制（每5ms执行）
    counter_angle += 1
    if counter_angle >= 5:
        counter_angle = 0
        vel_disturbance, angle_sum_error, angle_last_error = pid_controller(
            current_angle, target_angle,
            angle_kp, angle_ki, angle_kd,
            angle_sum_error, angle_last_error
        )
        vel_disturbance = max(min(vel_disturbance, 60), -60)

    target_vel = vel_disturbance
    # 内环控制
    delta_pwm, vel_last_error, vel_prev_error = pid_increment(
        pitch_vel, target_vel,
        vel_kp, vel_ki, vel_kd, 
        vel_last_error, vel_prev_error
    )

    pwm += delta_pwm
    pwm = max(min(pwm, 6000), -6000)
    
    turn_output = 0
    pwm_output = pwm  # 极性修改
    
    pwm_l = pwm_output + turn_output
    pwm_r = pwm_output - turn_output

    motor_l.duty(pwm_l)
    motor_r.duty(pwm_r)