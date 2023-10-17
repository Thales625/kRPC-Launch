import krpc
from time import sleep
from os import system

from Vector import Vector3

class Launch:
    def __init__(self):
        self.conn = krpc.connect('Launch Program')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        self.orbit = self.vessel.orbit
        self.body = self.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight = self.vessel.flight(self.body_ref)

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        self.stream_thrust = self.conn.add_stream(getattr, self.vessel, "thrust")
        self.stream_vel_body = self.conn.add_stream(getattr, self.flight, "velocity")
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.flight, "surface_altitude")
        self.stream_pitch = self.conn.add_stream(getattr, self.vessel.flight(self.surface_ref), "pitch")
        self.stream_dynamic_pressure = self.conn.add_stream(getattr, self.flight, "dynamic_pressure")
        self.stream_apoapsis = self.conn.add_stream(getattr, self.orbit, "apoapsis_altitude")
        
        # body properties
        self.surface_gravity = self.body.surface_gravity
        
        # Params
        self.state = 1
        self.target_twr = 4

        self.grav_turn_inclination = 90
        self.grav_turn_start = 500
        self.grav_turn_end = 60000
        self.grav_turn_max_apoapsis = 90000

        self.heading = 90

        # Initializing
        self.vessel.control.brakes = False
        self.vessel.control.rcs = False
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.reference_frame = self.surface_ref
        self.vessel.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
        self.vessel.auto_pilot.deceleration_time = (5, 5, 5)
        self.vessel.auto_pilot.target_roll = 0
        self.vessel.auto_pilot.target_direction = (1, 0, 0)

        if self.vessel.situation == self.vessel.situation.pre_launch:
            self.vessel.control.throttle = 1
            self.vessel.control.activate_next_stage()

        while True:
            sleep(0.01)

            # get streams
            vel = self.get_velocity()
            alt = self.get_altitude()
            mass = self.stream_mass()
            av_thrust = self.stream_av_thrust()
            thrust = self.stream_thrust()
            pitch = self.stream_pitch()
            dynamic_pressure = self.stream_dynamic_pressure()
            apoapsis = self.stream_apoapsis()
            
            twr = thrust / (self.surface_gravity * mass)
            a_eng = av_thrust / mass
            mag_speed = vel.magnitude()

            # display
            info_list = []
            info_list.append(('vel', mag_speed))
            info_list.append(('altitude', alt))
            info_list.append(('pitch', pitch))
            info_list.append(('mass', mass))
            info_list.append(('thrust', thrust))
            info_list.append(('acel', a_eng))
            info_list.append(('twr', twr))
            info_list.append(('pressure', dynamic_pressure))
            info_list.append(('apoapsis', apoapsis))
            info_list.append(('state', self.state))
            
            system('cls')
            self.show_info(info_list)

            # throttle controller
            throttle = (self.target_twr*mass*self.surface_gravity) / av_thrust
            self.vessel.control.throttle = throttle
                
            if self.state == 1: # grav turn
                if alt < self.grav_turn_end and apoapsis <= self.grav_turn_max_apoapsis:
                    if alt > self.grav_turn_start:
                        target_pitch = float((self.grav_turn_end - alt) * 90 / self.grav_turn_end)

                        self.vessel.auto_pilot.target_pitch_and_heading(target_pitch, self.heading)
                else:
                    self.state = 2
                    
            if self.state == 2: # end program
                self.vessel.control.throttle = 0

                print(f'{self.vessel.name} Reach apoapsis!')

                self.vessel.auto_pilot.disengage()
                self.vessel.control.rcs = False
                self.vessel.control.sas = True
                sleep(0.1)
                try:
                    self.vessel.control.sas_mode = self.vessel.control.sas_mode.prograde
                except:
                    self.vessel.auto_pilot.engage()
                    self.vessel.control.rcs = True
                    self.vessel.auto_pilot.target_direction = self.space_center.transform_direction(tuple(vel.normalize()), self.surface_ref, self.body_ref)
                    self.vessel.auto_pilot.wait()
                    self.vessel.control.rcs = False
                    self.vessel.auto_pilot.disengage()
                    
                self.conn.close()

                print('End launch program')
                break


    def show_info(self, info):
        text_size = max(map(lambda x: len(x[0]), info))

        result = "==================\n"
        for text, value in info:
            if len(text) > text_size:
                text = text[0:text_size]
            else:
                delta = text_size - len(text)
                for i in range(delta):
                    text += " "

            text += f': {value:.2f}\n'
            result += text
        result += "==================\n"

        print(result)

    def get_velocity(self):
        return Vector3(self.space_center.transform_direction(self.stream_vel_body(), self.body_ref, self.surface_ref))
    
    def get_altitude(self):
        return max(0, self.stream_surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])
    

if __name__ == '__main__':
    Launch()