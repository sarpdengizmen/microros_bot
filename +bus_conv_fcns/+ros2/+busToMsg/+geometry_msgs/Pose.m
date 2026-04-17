function rosmsgOut = Pose(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.position = bus_conv_fcns.ros2.busToMsg.geometry_msgs.Point(slBusIn.position,rosmsgOut.position(1));
    rosmsgOut.orientation = bus_conv_fcns.ros2.busToMsg.geometry_msgs.Quaternion(slBusIn.orientation,rosmsgOut.orientation(1));
end
