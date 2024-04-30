# Code for handling the kinematics of a robot with a single linear rail and 2 carriages that move along it 
# the toolhead is mounted at the intersection of a pair of arms that are driven by the carriages
#
# Copyright (C) 2024  Queron Williams <Ducktaperules@ooglemail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import stepper, mathutil

class LinearXYKinematics:
    def __init__(self, toolhead, config):
        # Setup  rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abz']

        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax = False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=a_endstop)
        rail_z = stepper.LookupMultiRail(
            stepper_configs[2], need_position_minmax = False,
            default_position_endstop=a_endstop)
        self.rails = [rail_a, rail_b, rail_z]

        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)

        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)

        # Read arm length and minimum atob distance
        self.arm_length = config.getfloat('arm_length', 1.0, above=0.)
        self.min_atob = config.getfloat('min_atob', 0.0, above=0.)

        # Setup boundary checks
        # acoords = list(zip(*self.anchors))
        # self.axes_min = toolhead.Coord(*[min(a) for a in acoords], e=0.)
        # self.axes_max = toolhead.Coord(*[max(a) for a in acoords], e=0.)
        # self.set_position([0., 0., 0.], ())
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        # get stepper positions
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        # x coord is midpoint between rail_a and rail_b
        xpos = (spos[0] + spos[1]) / 2
        # y coord is height above the rail
        ypos = math.sqrt(self.arm_length**2 - (spos[0] - spos[1])**2)
        # return x, y, z
        return [xpos, ypos, spos[2]]
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
    def home(self, homing_state):
        # XXX - homing not implemented
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])
    def check_move(self, move):
        # XXX - boundary checks and speed limits not implemented
        pass
    def get_status(self, eventtime):
        # XXX - homed_checks and rail limits not implemented
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return LinearXY(toolhead, config)
