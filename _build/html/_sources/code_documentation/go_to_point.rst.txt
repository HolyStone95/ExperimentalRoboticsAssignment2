
Code docs -- *go_to_point.py*
==============================

How to use this node -- service behaviour
-----------------------------------------

1. set the ROSParameters *des_pos_x* and *des_pos_y* in the parameter server using *rospy.set_param( "name", value )*
2. send *data=true* to the service ``/go_to_point`` of type ``std_srvs::SetBool``
3. wait until the server returns

*and here we go!*

code reference
--------------

.. automodule:: go_to_point
    :members:
    :noindex:

