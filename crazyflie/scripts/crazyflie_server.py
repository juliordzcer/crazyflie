#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from crazyflie.srv import *
from crazyflie.msg import *

import numpy as np

import time, sys
import math
from threading import Thread

# Importando las librerias de python de bitcraze.
import cflib
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils.power_switch import PowerSwitch


from threading import Thread


class CrazyflieROS:
    Disconnected = 0
    Connecting = 1
    Connected = 2

    def __init__(self, link_uri, tf_prefix, roll_trim, pitch_trim ,enable_logging):
        self.link_uri = link_uri
        self.tf_prefix = tf_prefix
        self.roll_trim = roll_trim
        self.pitch_trim = pitch_trim
        self.enable_logging = enable_logging
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.link_quality_updated.add_callback(self._link_quality_updated)

        self._subCmdFull = rospy.Subscriber(tf_prefix + "/cmd_full", Full, self._cmdsetpointChanged)
        self._subCmdExtPose = rospy.Subscriber(tf_prefix + "/external_pose", PoseStamped, self._poseMeasurementChanged)
        self._pubSignals = rospy.Publisher(tf_prefix + "/sc", Twist, queue_size=10)

        self._state = CrazyflieROS.Disconnected
        # Services
        rospy.Service(tf_prefix + "/update_params", UpdateParams, self._update_params)
        rospy.Service(tf_prefix + "/emergency", Empty, self._emergency)

        self._isEmergency = False

        Thread(target=self._update).start()

    def _try_to_connect(self):
        rospy.loginfo("Connecting to %s" % self.link_uri)
        self._state = CrazyflieROS.Connecting
        self._cf.open_link(self.link_uri)

    def _connected(self, link_uri):

        rospy.loginfo("Connected to %s" % link_uri)
        self._state = CrazyflieROS.Connected

        if self.enable_logging:
            self._lg_signals = LogConfig(name="Signals_n", period_in_ms=10)

            self._lg_signals.add_variable("signals_n.tau_phi"  , "float")
            self._lg_signals.add_variable("signals_n.tau_theta", "float")
            self._lg_signals.add_variable("signals_n.tau_psi"  , "float")
            self._lg_signals.add_variable("signals_n.u"        , "float")
            
            try:
                self._cf.log.add_config(self._lg_signals)
                self._lg_signals.data_received_cb.add_callback(self._log_data_Signal)
                self._lg_signals.error_cb.add_callback(self._log_error)
                self._lg_signals.start()
            except KeyError as e:
                rospy.logwarn('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))

                print()
            except AttributeError:
                rospy.logfatal("Could not add logconfig since some variables are not in TOC")


        p_toc = self._cf.param.toc.toc
        for group in p_toc.keys():
            self._cf.param.add_update_callback(group=group, name=None, cb=self._param_callback)
            for name in p_toc[group].keys():
                ros_param = "/{}/{}/{}".format(self.tf_prefix, group, name)
                cf_param = "{}.{}".format(group, name)
                if rospy.has_param(ros_param):
                    self._cf.param.set_value(cf_param, rospy.get_param(ros_param))
                else:
                    self._cf.param.request_param_update(cf_param)

    def _connection_failed(self, link_uri, msg):
        rospy.logfatal("Connection to %s failed: %s" % (link_uri, msg))
        self._cf.close_link()
        self._state = CrazyflieROS.Disconnected

    def _connection_lost(self, link_uri, msg):
        rospy.logfatal("Connection to %s lost: %s" % (link_uri, msg))
        self._state = CrazyflieROS.Disconnected

    def _disconnected(self, link_uri):
        rospy.logfatal("Disconnected from %s" % link_uri)
        self._state = CrazyflieROS.Disconnected


    def _link_quality_updated(self, percentage):
        if percentage < 80:
            rospy.logwarn("Connection quality is: %f" % (percentage))

    def _log_error(self, logconf, msg):
        rospy.logfatal("Error when logging %s: %s" % (logconf.name, msg))

    def _log_data_Signal(self, timestamp, data, logconf):
        msg = Twist()
        msg.linear.x = data["signals_n.tau_phi"]
        msg.linear.y = data["signals_n.tau_theta"]
        msg.linear.z = data["signals_n.tau_psi"]
        msg.linear.z = data["signals_n.u"]

        self._pubSignals.publish(msg)

    def _cmdsetpointChanged(self,msg):
        x = msg.twist1.linear.x
        y = msg.twist1.linear.y
        z = msg.twist1.linear.z
        psi = msg.twist2.angular.x

        self._cf.commander.send_position_setpoint(x, y, z, psi)

    def _poseMeasurementChanged(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        self._cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
        

    def _param_callback(self, name, value):
        ros_param = "{}/{}".format(self.tf_prefix, name.replace(".", "/"))
        rospy.set_param(ros_param, value)

    def _update_params(self, req):
        rospy.loginfo("Update parameters %s" % (str(req.params)))
        for param in req.params:
            ros_param = "/{}/{}".format(self.tf_prefix, param)
            cf_param = param.replace("/", ".")
            print(cf_param)
            #if rospy.has_param(ros_param):
            self._cf.param.set_value(cf_param, str(rospy.get_param(ros_param)))
        return UpdateParamsResponse()

    def _emergency(self, req):
        rospy.logfatal("Emergency requested!")
        self._isEmergency = True
        self._cf.loc.send_emergency_stop()
        return EmptyResponse()



    def _update(self):
        while not rospy.is_shutdown():
            if self._isEmergency:
                break
            if self._state == CrazyflieROS.Disconnected:
                self._try_to_connect()
            elif self._state == CrazyflieROS.Connected:
                rospy.sleep(0.2)
            else:
                rospy.sleep(0.5)
        rospy.sleep(0.1)
        self._cf.close_link()



def add_crazyflie(req):
    rospy.loginfo("Adding %s as %s with trim(%f, %f). Logging: %s" % (req.uri, req.tf_prefix, req.roll_trim, req.pitch_trim, str(req.enable_logging)))
    CrazyflieROS(req.uri, req.tf_prefix, req.roll_trim, req.pitch_trim, req.enable_logging)
    return AddCrazyflieResponse()

if __name__ == '__main__':
    rospy.init_node('crazyflie_server')

    cflib.crtp.init_drivers()

    rospy.Service("add_crazyflie", AddCrazyflie, add_crazyflie)
    rospy.spin()

