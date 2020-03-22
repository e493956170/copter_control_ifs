# copter_control_ifs
copter control interface with dji or mavlink devicce like px4/ardupilot

# Introduction
A C++ based copter control interface to simplify the repeated work in copter control . People should be easily use this to devolop slam,trajectory planning ,or other state-of-art things.


# FlightTask

Flight tasks manager is develop to virtual funtion ,so it is easy to extend other motion group by using the api.More work will be done.

# RouteTracker

RouteTracker is based on pure pursuit.

# Visualization
Visualization in rviz has been wrapped into some simple steps by template class.So it will be more convenient to show your hard work.
More visualiztion wrapper of rviz  is under working . Also ,the purpose of this is to simplify work not to heavy study content.


