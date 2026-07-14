function stopMotors(mA,mB,mC)
% Sends stop signals to the motors mA, mB, mC
    try, stop(mA); end %#ok<*TRYNC>
    try, stop(mB); end
    try, stop(mC); end
end