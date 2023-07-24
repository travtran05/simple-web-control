from motor_test import test_motor
import time
from pymavlink import mavutil
import socket
import select

def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")

def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")

def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    start_time = time.time()
    while time.time()-start_time < seconds:
        for i in range(len(motor_settings)):
            test_motor(mav_connection=mav_connection, motor_id=i, power=motor_settings[i])
        time.sleep(0.2)


#socket

'''

if __name__ == "__main__":
    HOST = "10.29.120.78"  # The server's hostname or IP address
    PORT = 8888  # The port used by the server

    # first 6 numbers are thrusters 1-6 and 7th is time


    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("10.29.120.78", 8566)) # associates socket with a specific network interface and port number
        s.listen() # makes this a listening socket that enables the server to accept connections
        conn, addr = s.accept() # blocks execution and waits for an incoming connection
        with conn:
            print(f"Connected by {addr}")
            exit = False
            mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
            mav_connection.wait_heartbeat()
            arm_rov(mav_connection)
            s.setblocking(False)
            while exit == False:
                try:
                    data = conn.recv(1024 , socket.MSG_DONTWAIT)
                    input = data.decode("utf-8")
                except BlockingIOError:
                    input = "0 0 0 0 0 0 1"
                if(input.strip() == "done"):
                    conn.send("Disarmed!".encode())
                    exit = True
                else:
                    settings = input.split(" ")
                    run = True
                    if(len(settings) != 7):
                        run = False
                    else:
                        for i in range(0, 7):
                            try:
                                if(i == 6 and int(settings[i]) <= 0):
                                    run = False
                                elif(i != 6 and (int(settings[i]) < -100 or int(settings[i]) > 100)):
                                    run = False
                            except ValueError or TypeError:
                                run = False
                                break
                    if(run):
                        run_motors_timed(mav_connection, seconds=int(settings[6]), motor_settings=[int(settings[0]), int(settings[1]), int(settings[2]), int(settings[3]), int(settings[4]), int(settings[5])])
                    else:
                        error = "Invalid Input\n"
                        print(error)
                        conn.send(error.encode())
        s.close()

'''




#main

if __name__ == "__main__":

    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    print("Waiting for connection")
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    print("ARMED")

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
    sec = 3.8325
   
    for i in range(4):
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[-100,-100 ,0 ,100, 0, 0])
        print("command one")
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,100,0,100, 0, 0])
        print("command two")

        run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,0 ,-100 ,-100, 0, 0])
        print("command three")
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,0, 100,100, 0, 0])
        print("command four")
       
    run_motors_timed(mav_connection, seconds=5.825, motor_settings=[100, 0 ,0 ,100, 0, 0])
    print("turn")


    for i in range(4):
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[-100,-100 ,0 ,100, 0, 0])
        print("command one")
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,100,0,100, 0, 0])
        print("command two")

        run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,0 ,-100 ,-100, 0, 0])
        print("command three")
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,0, 100,100, 0, 0])
        print("command four")

    # stop
    run_motors_timed(mav_connection, seconds=5, motor_settings=[0, 0, 0, 0, 0, 0])
    print("stopped")
    ####
    # Disarm ROV and exit
    ####
    disarm_rov(mav_connection)
    print("disarmed")



    