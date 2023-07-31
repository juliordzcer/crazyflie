#!/usr/bin/env python3

import rospy

def update_parameters():
    rospy.init_node('update_params_node', anonymous=True)

    # Obtener los valores de los parámetros desde ROS
    crazyflie_namespace = rospy.get_param('~frame', ' ')
    controller_type = rospy.get_param('~n', 2)

    # Actualizar los parámetros en el espacio de nombres especificado
    try:
        rospy.set_param(crazyflie_namespace + "/stabilizer/estimator", 2)
        rospy.set_param(crazyflie_namespace + "/stabilizer/controller", controller_type)
        rospy.set_param(crazyflie_namespace + "/locSrv/extPosStdDev", 1e-3)
        rospy.set_param(crazyflie_namespace + "/locSrv/extQuatStdDev", 0.5e-1)
        rospy.set_param(crazyflie_namespace + "/kalman/resetEstimation", 1)
        rospy.loginfo('Parámetros actualizados para el espacio de nombres: %s', crazyflie_namespace)
    except rospy.ROSException as e:
        rospy.logerr('Error al actualizar los parámetros: %s', str(e))

if __name__ == '__main__':
    rospy.sleep(7)
    update_parameters()
