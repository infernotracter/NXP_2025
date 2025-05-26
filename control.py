def pid_controller(now, target, kp, ki, kd, sum_err, last_err):
    error = target - now
    
    sum_err += error
    
    derivative = error - last_err
    
    I = max(min(sum_err, 2000.0), -2000)
    
    output = kp * error + ki *sum_err + kd * derivative
    
    return output, sum_err, error

def pid_increment(now, target, kp, ki, kd, last_err, prev_err):
    error = target - now
    
    increment = kp*(error - last_err) + ki*error + kd*(error - 2*last_err + prev_err)
    
    last_err=error
    
    prev_err=last_err
    
    return increment, last_err, prev_err


def vel_loop_callback(pit0):
    