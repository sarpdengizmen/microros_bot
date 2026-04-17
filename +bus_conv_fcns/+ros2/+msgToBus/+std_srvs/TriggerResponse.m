function slBusOut = TriggerResponse(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.success = logical(msgIn.success);
    slBusOut.message_SL_Info.ReceivedLength = uint32(strlength(msgIn.message));
    currlen  = min(slBusOut.message_SL_Info.ReceivedLength, length(slBusOut.message));
    slBusOut.message_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.message(1:currlen) = uint8(char(msgIn.message(1:currlen))).';
end
