function rosmsgOut = Time(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.sec = int32(slBusIn.sec);
    rosmsgOut.nanosec = uint32(slBusIn.nanosec);
end
