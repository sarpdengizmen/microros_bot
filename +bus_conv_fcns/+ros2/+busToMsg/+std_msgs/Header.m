function rosmsgOut = Header(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.stamp = bus_conv_fcns.ros2.busToMsg.builtin_interfaces.Time(slBusIn.stamp,rosmsgOut.stamp(1));
    rosmsgOut.frame_id = char(slBusIn.frame_id);
    if slBusIn.frame_id_SL_Info.CurrentLength < numel(slBusIn.frame_id)
    rosmsgOut.frame_id(slBusIn.frame_id_SL_Info.CurrentLength+1:numel(slBusIn.frame_id)) = [];
    end
end
