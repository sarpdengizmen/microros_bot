function rosmsgOut = PoseStamped(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.header = bus_conv_fcns.ros2.busToMsg.std_msgs.Header(slBusIn.header,rosmsgOut.header(1));
    rosmsgOut.pose = bus_conv_fcns.ros2.busToMsg.geometry_msgs.Pose(slBusIn.pose,rosmsgOut.pose(1));
end
