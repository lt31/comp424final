import time

class PWMControl:
   
    ''' initialization '''
    def __init__(self):
        self.jumped = False
        self.speed = open("/sys/module/gpiod_driver/parameters/rot_time", "r")
        # self.last_rot = open("/sys/module/gpiod_driver/parameters/last_time", "r")
        self.cur_throttle = 7.5
        # P9_14 - Speed/ESC (7.75%)
        with open('/dev/bone/pwm/1/a/period', 'w') as filetowrite:
            filetowrite.write('20000000')
        with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
            filetowrite.write('1500000')
        with open('/dev/bone/pwm/1/a/enable', 'w') as filetowrite:
            filetowrite.write('1')
        # P9_16 - Steering (7.5%)
        with open('/dev/bone/pwm/1/b/period', 'w') as filetowrite:
            filetowrite.write('20000000')
        with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
            filetowrite.write('1500000')
        with open('/dev/bone/pwm/1/b/enable', 'w') as filetowrite:
            filetowrite.write('1')

    ''' gets the speed of the RC car '''
    def get_speed(self):
        self.speed.seek(0)
        cur_speed = float(self.speed.read())
        with open('/dev/meschar', 'r') as reset:
            reset.read()
        return cur_speed

    # def get_last_rot(self):
    #     self.last_rot.seek(0)
    #     return int(self.last_rot.read())

    def percent_to_period(self, percent: float):
        return str(int(percent * 200000))

    ''' set throttle value given PWM percent '''
    def set_throttle_direct(self, percent):
        self.cur_throttle = percent
        with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as throttle:
            # print(self.percent_to_period(percent))
            throttle.write(self.percent_to_period(percent))

    ''' set throttle value given rotational period for constant speed '''
    def set_throttle(self, rot_period: float):
        # if vehicle was previously stopped
        if rot_period == 0:
            print("rot_period 0")
            self.jumped = False

            self.set_throttle_direct(7.9)
            return 7.9

        # if vehicle was not previously stopped
        cur_rot_period = self.get_speed()
        print(cur_rot_period)
        diff = abs(cur_rot_period - rot_period)

        jump = 0
        if cur_rot_period == 1000000 and not self.jumped:
            print("Jumped")
            self.jumped = True
            jump = 0.05

        delta = 0.001
        # print(time.time_ns() / 1000000)
        # print(self.get_last_rot())
        
        # if time.time_ns() / 1000000 - self.get_last_rot() > 250:
        #     self.set_throttle_direct(self.cur_throttle + delta)
        #     print(f"Manually increasing: {self.cur_throttle}")
        #     return

        # Speed is changed based on throttle error  
        if diff < 1:
            pass
        elif rot_period > cur_rot_period:
            self.set_throttle_direct(self.cur_throttle - (jump + self.diff_to_delta(diff)))
            print(f"Decreasing: {self.cur_throttle}")
            return self.cur_throttle
        elif rot_period < cur_rot_period:
            self.set_throttle_direct(self.cur_throttle + (jump + self.diff_to_delta(diff)))
            print(f"Increasing: {self.cur_throttle}")
            return self.cur_throttle

    ''' detect throttle error '''
    def diff_to_delta(self, diff):
        if diff < 30:
            return -0.0005
        elif diff < 25:
            return 0.0001
        elif diff < 400:
            return 0.0001
        else:
            return 0.0003

    ''' set steering '''
    def set_steering(self, percent: float):
        with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as steering:
            steering.write(self.percent_to_period(percent))
        return percent

    ''' shutdown '''
    def shutdown(self):
        print("Shutting down PWM...")   
        # Center steering and stop motor
        self.set_throttle_direct(7.5)
        self.set_steering(7.5)
        time.sleep(0.2)
        # Shut down throttle
        with open('/dev/bone/pwm/1/a/enable', 'w') as filetowrite:
            filetowrite.write('1')

        # Shut down steering
        with open('/dev/bone/pwm/1/b/enable', 'w') as filetowrite:
            filetowrite.write('0')

        # Close speed file
        self.speed.close()
        # self.last_rot.close()

# PWM testing
if __name__ == "__main__":
    pwm = PWMControl()
    import signal, sys, select

    time.sleep(3)
    # pwm.set_steering(9)
    pwm.set_throttle_direct(8)

    # print("9")
    # pwm.set_steering(9)
    # time.sleep(3)
    # print("6")
    # pwm.set_steering(6)
    # time.sleep(3)
    # print("7.5")
    # pwm.set_steering(7.5)

    run = True

    def stop(signum, stackframe):
        global run
        print("Stopping...")
        run = False

    signal.signal(signal.SIGINT, stop)

    # start = time.time()
    # while run:
    #     if time.time() - start > 3:
    #         break
    target = 0
    while run:
        i, o, e = select.select( [sys.stdin], [], [], 0.01 )
        if i:
            try:
                target = int(sys.stdin.readline().strip())
            except ValueError:
                pass
        pwm.set_throttle(target)
        time.sleep(0.1)

    pwm.shutdown()