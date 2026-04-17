function rosmsgOut = Quaternion(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.x = double(slBusIn.x);
    rosmsgOut.y = double(slBusIn.y);
    rosmsgOut.z = double(slBusIn.z);
    rosmsgOut.w = double(slBusIn.w);
end
